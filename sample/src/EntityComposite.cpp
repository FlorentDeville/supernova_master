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

#include "snRigidbody.h"
#include "snICollider.h"
#include "snOBB.h"
#include "snSphere.h"
#include "snCapsule.h"
#include "snVec.h"

#include "Graphics.h"
#include "Camera.h"
#include "D3D.h"
#include "GfxEntitySphere.h"
using namespace DirectX;

namespace Devil
{
	EntityComposite::EntityComposite(snRigidbody* _actor, const XMFLOAT4& _color)
	{
		m_color = XMVectorSet(_color.x, _color.y, _color.z, _color.w);
		m_actor = _actor;
		vector<snICollider*> colliders = m_actor->getColliders();

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
		vector<snICollider*> colliders = m_actor->getColliders();
		for (vector<snICollider*>::const_iterator i = colliders.cbegin(); i != colliders.cend(); ++i)
		{
			snMatrix44f temp = (*i)->getTransform().getLocalToWorld();

			snMatrix44f scale;
			IGfxEntity* gfx = 0;
			switch ((*i)->getTypeOfCollider())
			{
				case snEColliderType::snEColliderBox:
				{
					snOBB* box = static_cast<snOBB*>((*i));
					scale.createScale(box->getExtends() * 2);
					gfx = GRAPHICS->getBox();
				}
				break;

				case snEColliderType::snEColliderSphere:
				{
					snSphere* sphere = static_cast<snSphere*>((*i));
					scale.createScale(sphere->getRadius() * 2);
					gfx = GRAPHICS->getSphere();
				}
				break;

				case snEColliderType::snEColliderCapsule:
				{
					snCapsule* capsule = static_cast<snCapsule*>((*i));
					float diameter = capsule->getRadius() * 2;

					//draw the first sphere
					scale.createScale(diameter);
					snMatrix44f translate;
					translate.createTranslation(capsule->getFirstEndPoint());

					snMatrix44f worldTransform;
					snMatrixMultiply4(scale, translate, worldTransform);
					XMMATRIX dxWorldMatrix;
					dxWorldMatrix.r[0] = worldTransform.m_r[0];
					dxWorldMatrix.r[1] = worldTransform.m_r[1];
					dxWorldMatrix.r[2] = worldTransform.m_r[2];
					dxWorldMatrix.r[3] = worldTransform.m_r[3];

					gfx = GRAPHICS->getSphere();
					gfx->render(dxWorldMatrix, viewMatrix, projectionMatrix, m_color, m_texture, m_wireframe);

					//draw the second sphere
					translate.createTranslation(capsule->getSecondEndPoint());
					snMatrixMultiply4(scale, translate, worldTransform);
					dxWorldMatrix.r[0] = worldTransform.m_r[0];
					dxWorldMatrix.r[1] = worldTransform.m_r[1];
					dxWorldMatrix.r[2] = worldTransform.m_r[2];
					dxWorldMatrix.r[3] = worldTransform.m_r[3];
					gfx->render(dxWorldMatrix, viewMatrix, projectionMatrix, m_color, m_texture, m_wireframe);

					//prepare the capsule
					float length = Supernova::Vector::snVec3Norme(capsule->getFirstEndPoint() - capsule->getSecondEndPoint());
					scale.createScale(Supernova::Vector::snVec4Set(capsule->getRadius(), length, capsule->getRadius(), 1));
					gfx = GRAPHICS->getCylinder();
				}
				break;

				default:
					continue;
			}

			snMatrix44f worldTransform;
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