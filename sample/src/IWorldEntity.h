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
#ifndef I_WORLD_ENTITY_H
#define I_WORLD_ENTITY_H

#include <DirectXMath.h>
using DirectX::XMVECTOR;
using DirectX::XMFLOAT3;

//#include "snActor.h"
namespace Supernova
{
	class snIActor;
}
using namespace Supernova;

#include <vector>
using std::vector;

#include "IComponent.h"

namespace Devil
{
	class IWorldEntity
	{
	protected:
		XMVECTOR m_position;
		XMFLOAT3 m_orientation;
		XMFLOAT3 m_scale;

		/*actor representing this entity in the physics engine*/
		snIActor* m_actor;

		/*Indicates if the entity is active. A not active entity is not updated nor rendered.*/
		bool m_isActive;

		//List of components attached to this entity and to update after updating the entity.
		vector<IComponent*> m_postUpdateComponents;

		//List of components to update before updating the entity.
		vector<IComponent*> m_preUpdateComponents;

		//Indicates if this entity has to be rendered in wireframe
		bool m_wireframe;

	public:
		IWorldEntity() : m_isActive(true), m_postUpdateComponents(), m_preUpdateComponents(), m_wireframe(false)
		{
			m_position = DirectX::XMVectorSet(0, 0, 0, 1);
			m_orientation = XMFLOAT3(0, 0, 0);
			m_scale = XMFLOAT3(1, 1, 1);
		};

		virtual ~IWorldEntity()
		{
			for (vector<IComponent*>::iterator i = m_preUpdateComponents.begin(); i != m_preUpdateComponents.end(); ++i)
				delete *i;
			m_preUpdateComponents.clear();

			for (vector<IComponent*>::iterator i = m_postUpdateComponents.begin(); i != m_postUpdateComponents.end(); ++i)
				delete *i;
			m_postUpdateComponents.clear();
		};

		virtual void update() = 0;
		virtual void render() = 0;

		virtual void spriteRender(){};

		void setPosition(const XMVECTOR& _position)
		{ 
			m_position = _position; 
		}

		void setOrientation(const XMFLOAT3& _orientation){ m_orientation = _orientation; }

		void setScaling(const XMFLOAT3& _scaling){ m_scale = _scaling; }

		void setActor(snIActor* _actor){ m_actor = _actor; }

		void setIsActive(bool _isActive){ m_isActive = _isActive; }

		void setWireframe(bool _isWireframe){ m_wireframe = _isWireframe; }

		bool getIsActive() const { return m_isActive; }

		snIActor* getActor() const { return m_actor; }

		//Return the list of components attached to this entity
		vector<IComponent*> const & getPostUpdateComponents() { return m_postUpdateComponents; }

		//Return the list of components to update before the entity
		vector<IComponent*> const & getPreUpdateComponents() { return m_preUpdateComponents; }

		//Return the position of the entity.
		const XMVECTOR& getPosition() const { return m_position; }

		//Add a component to the entity
		void addPostUpdateComponent(IComponent* _newComponent){ m_postUpdateComponents.push_back(_newComponent); }

		//Add a component to the entity. The component will be updated before the entity
		void addPreUpdateComponent(IComponent* _newComponent){ m_preUpdateComponents.push_back(_newComponent); }

		void* operator new(size_t _count)
		{
			return _aligned_malloc(_count, 16);
		}

		void operator delete(void* _p)
		{
			_aligned_free(_p);
		}

		//Updat all the components of the entity
		void postUpdateComponents(float _dt)
		{
			for (vector<IComponent*>::iterator i = m_postUpdateComponents.begin(); i != m_postUpdateComponents.end(); ++i)
			{
				if ((*i)->getIsActive())
					(*i)->update(_dt);
			}
		}

		void preUpdateComponents(float _dt)
		{
			for (vector<IComponent*>::iterator i = m_preUpdateComponents.begin(); i != m_preUpdateComponents.end(); ++i)
			{
				if ((*i)->getIsActive())
					(*i)->update(_dt);
			}
		}

		void postRenderComponents()
		{
			for (vector<IComponent*>::iterator i = m_postUpdateComponents.begin(); i != m_postUpdateComponents.end(); ++i)
			{
				if ((*i)->getIsActive())
					(*i)->render();
			}
		}

		void preRenderComponents()
		{
			for (vector<IComponent*>::iterator i = m_preUpdateComponents.begin(); i != m_preUpdateComponents.end(); ++i)
			{
				if ((*i)->getIsActive())
					(*i)->render();
			}
		}
	};
}

#endif //I_WORLD_ENTITY_H
