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

#include "EntityCharacter.h"

#include "snMatrix44f.h"

#include "Graphics.h"
#include "Camera.h"
#include "D3D.h"

#include "World.h"
#include "snScene.h"
#include "snRay.h"

#include "Input.h"
#include "EntityCamera.h"

//using namespace Supernova::Vector;
using namespace DirectX;

namespace Devil
{
	namespace Worlds
	{
		namespace Entities
		{
			EntityCharacter::EntityCharacter()
			{
				m_radius = 1;
				m_height = 2;
				m_color = DirectX::XMVectorSet(1, 1, 1, 1);
			}

			EntityCharacter::~EntityCharacter()
			{}

			bool EntityCharacter::initialize(float _radius, float _height, const XMVECTOR& _color)
			{
				m_radius = _radius;
				m_height = _height;
				m_color = _color;

				return true;
			}

			void EntityCharacter::update()
			{
				//Compute new position
				float amountV = INPUT->getMessage(Devil::Input::InputMessage::MOVE_UP_AND_DOWN);
				float amountH = INPUT->getMessage(Devil::Input::InputMessage::MOVE_SIDEWAY);

				float speedMultiplicator = INPUT->getMessage(Devil::Input::InputMessage::MOVE_FORWARD) + 1;

				snVec forward = m_position - WORLD->getCamera()->getPosition();//Vector::snVec4Set(0, 0, 1, 0);
				forward.m128_f32[VEC_ID_Y] = 0;
				Vector::snVec3Normalize(forward);

				snVec up = Vector::snVec4Set(0, 1, 0, 0);
				snVec right = Vector::snVec3Cross(up, forward); 

				m_position = m_position + (forward * amountV + right * amountH) * speedMultiplicator;

				//Use ryacast to find correct elevation
				snRay ray(m_position, Vector::snVec4Set(0, -1, 0, 0));

				snVec hit;
				bool res = WORLD->getPhysicsScene()->raycast(ray, hit);

				if(!res)
				{
					return;
				}

				m_position.m128_f32[VEC_ID_Y] = hit.m128_f32[VEC_ID_Y] + m_radius + m_height * 0.5f;
			}

			void EntityCharacter::render()
			{
				//get view and projection matrices
				XMMATRIX viewMatrix, projectionMatrix;
				GRAPHICS->getCamera()->GetViewMatrix(viewMatrix);
				GRAPHICS->getDirectXWrapper()->getProjectionMatrix(projectionMatrix);

				//draw the first sphere
				snMatrix44f scale;
				scale.createScale(m_radius * 2);
				snMatrix44f translate;
				translate.createTranslation(m_position + Vector::snVec4Set(0, m_height * 0.5f, 0, 0));

				snMatrix44f worldTransform;
				snMatrixMultiply4(scale, translate, worldTransform);
				XMMATRIX dxWorldMatrix;
				dxWorldMatrix.r[0] = worldTransform.m_r[0];
				dxWorldMatrix.r[1] = worldTransform.m_r[1];
				dxWorldMatrix.r[2] = worldTransform.m_r[2];
				dxWorldMatrix.r[3] = worldTransform.m_r[3];

				IGfxEntity* gfx = GRAPHICS->getSphere();
				gfx->render(dxWorldMatrix, viewMatrix, projectionMatrix, m_color, m_texture, m_wireframe);

				//draw the second sphere
				translate.createTranslation(m_position - Vector::snVec4Set(0, m_height * 0.5f, 0, 0));
				snMatrixMultiply4(scale, translate, worldTransform);
				dxWorldMatrix.r[0] = worldTransform.m_r[0];
				dxWorldMatrix.r[1] = worldTransform.m_r[1];
				dxWorldMatrix.r[2] = worldTransform.m_r[2];
				dxWorldMatrix.r[3] = worldTransform.m_r[3];
				gfx->render(dxWorldMatrix, viewMatrix, projectionMatrix, m_color, m_texture, m_wireframe);

				//draw the cylinder
				scale.createScale(Supernova::Vector::snVec4Set(m_radius, m_height, m_radius, 1));
				translate.createTranslation(m_position);
				gfx = GRAPHICS->getCylinder();
				snMatrixMultiply4(scale, translate, worldTransform);
				dxWorldMatrix.r[0] = worldTransform.m_r[0];
				dxWorldMatrix.r[1] = worldTransform.m_r[1];
				dxWorldMatrix.r[2] = worldTransform.m_r[2];
				dxWorldMatrix.r[3] = worldTransform.m_r[3];
				gfx->render(dxWorldMatrix, viewMatrix, projectionMatrix, m_color, m_texture, m_wireframe);
			}

			void* EntityCharacter::operator new(size_t _count)
			{
				return _aligned_malloc(_count, 16);
			}

			void EntityCharacter::operator delete(void* _p)
			{
				_aligned_free(_p);
			}
		}
	}
}