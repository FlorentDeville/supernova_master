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
#include "snColliderBox.h"
#include "snVec.h"
using namespace Supernova::Vector;

#include "Graphics.h"
#include "Camera.h"
#include "D3D.h"
using namespace DirectX;

namespace Devil
{
	EntityComposite::EntityComposite(snIActor* _actor, const XMFLOAT4& _color)
	{
		m_actor = _actor;
		vector<snICollider*> colliders = m_actor->getColliders();

		for (vector<snICollider*>::const_iterator i = colliders.cbegin(); i != colliders.cend(); ++i)
		{
			switch ((*i)->getTypeOfCollider())
			{
			case snEColliderType::snEColliderBox:
				snColliderBox* box = static_cast<snColliderBox*>(*i);
				snVec size = box->getSize();
				GfxEntityBox* gfxBox = GRAPHICS->createBox(XMFLOAT3(snVec4GetX(size), snVec4GetY(size), snVec4GetZ(size)), _color);
				m_gfx.push_back((IGfxEntity*)gfxBox);

				break;
			}
		}
		
	}

	EntityComposite::~EntityComposite()
	{

	}

	void EntityComposite::update()
	{

	}

	void EntityComposite::render()
	{
		//get view and projection matrices
		XMMATRIX viewMatrix, projectionMatrix;
		GRAPHICS->getCamera()->GetViewMatrix(viewMatrix);
		GRAPHICS->getDirectXWrapper()->getProjectionMatrix(projectionMatrix);

		//compute position and orientation of the actor
		vector<snICollider*> colliders = m_actor->getColliders();
		vector<snICollider*>::const_iterator currentCollider = colliders.cbegin();

		XMMATRIX translation = XMMatrixTranslationFromVector(m_actor->getPosition());
		XMMATRIX orientation;
		orientation.r[0] = m_actor->getOrientationMatrix().m_r[0];
		orientation.r[1] = m_actor->getOrientationMatrix().m_r[1];
		orientation.r[2] = m_actor->getOrientationMatrix().m_r[2];
		orientation.r[3] = m_actor->getOrientationMatrix().m_r[3];

		XMMATRIX transform = orientation * translation;

		//loop through each collider to display them using their offsets.
		for (vector<IGfxEntity*>::const_iterator i = m_gfx.cbegin(); i != m_gfx.cend(); ++i)
		{
			XMMATRIX offset = XMMatrixTranslationFromVector((*currentCollider)->getOrigin());
			transform = offset * orientation * translation;

			(*i)->render(transform, viewMatrix, projectionMatrix);

			++currentCollider;
		}
	}
}