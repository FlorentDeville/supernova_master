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

#include "EntityComposite.h"

#include "snIActor.h"
#include "snICollider.h"
#include "snColliderContainer.h"
#include "snOBB.h"
#include "snColliderSphere.h"
#include "snVec.h"
using namespace Supernova::Vector;

#include "Graphics.h"
#include "Camera.h"
#include "D3D.h"
#include "GfxEntitySphere.h"
using namespace DirectX;

namespace Devil
{
	EntityComposite::EntityComposite(snIActor* _actor, const XMFLOAT4& _color)
	{
		m_color = XMVectorSet(_color.x, _color.y, _color.z, _color.w);
		m_actor = _actor;
		vector<snColliderContainer*> colliders = m_actor->getColliders();

		//create a sphere for the center of mass
		m_gfxCenterOfMass = GRAPHICS->getSphere();

		
	}

	EntityComposite::~EntityComposite()
	{

	}

	void EntityComposite::update()
	{
		m_position = m_actor->getPosition();
	}

	void EntityComposite::render()
	{
		//get view and projection matrices
		XMMATRIX viewMatrix, projectionMatrix;
		GRAPHICS->getCamera()->GetViewMatrix(viewMatrix);
		GRAPHICS->getDirectXWrapper()->getProjectionMatrix(projectionMatrix);

		//compute position and orientation of the actor
		vector<snColliderContainer*> colliders = m_actor->getColliders();

		snMatrix44f globalTransform;
		snMatrixCreateTransform(m_actor->getOrientationMatrix(), m_actor->getPosition(), globalTransform);


		for (vector<snColliderContainer*>::const_iterator i = colliders.cbegin(); i != colliders.cend(); ++i)
		{
			snMatrix44f worldTransform, temp;
			snMatrixMultiply4((*i)->m_localTransform, globalTransform, temp);

			snMatrix44f scale;
			IGfxEntity* gfx = 0;
			switch ((*i)->m_collider->getTypeOfCollider())
			{
				case snEColliderType::snEColliderBox:
				{
					snOBB* box = static_cast<snOBB*>((*i)->m_collider);
					scale.createScale(box->getSize());
					gfx = GRAPHICS->getBox();
				}
				break;

				case snEColliderType::snEColliderSphere:
				{
					snColliderSphere* sphere = static_cast<snColliderSphere*>((*i)->m_collider);
					scale.createScale(sphere->getRadius() * 2);
					gfx = GRAPHICS->getSphere();
				}
				break;

				default:
					continue;
			}

			snMatrixMultiply4(scale, temp, worldTransform);
			XMMATRIX dxWorldMatrix;
			dxWorldMatrix.r[0] = worldTransform.m_r[0];
			dxWorldMatrix.r[1] = worldTransform.m_r[1];
			dxWorldMatrix.r[2] = worldTransform.m_r[2];
			dxWorldMatrix.r[3] = worldTransform.m_r[3];
			gfx->render(dxWorldMatrix, viewMatrix, projectionMatrix, m_color, m_texture, m_wireframe);
		}

		//render the center of mass
		XMMATRIX centerOfMassPosition = XMMatrixIdentity();
		centerOfMassPosition.r[3] = m_actor->getWorldCenterOfMass();
		m_gfxCenterOfMass->render(centerOfMassPosition, viewMatrix, projectionMatrix, Colors::Green, m_texture, m_wireframe);
	}
}