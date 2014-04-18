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
using namespace DirectX;

//#include "snActor.h"
namespace Supernova
{
	class snActor;
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
		snActor* m_actor;

		/*Indicates if the entity is active. A not active entity is not updated nor rendered.*/
		bool m_isActive;

		//List of components attached to this entity.
		vector<IComponent*> m_components;

	public:
		IWorldEntity() : m_isActive(true), m_components()
		{
			m_position = XMVectorSet(0, 0, 0, 1);
			m_orientation = XMFLOAT3(0, 0, 0);
			m_scale = XMFLOAT3(1, 1, 1);
		};

		virtual ~IWorldEntity(){};
		virtual void update() = 0;
		virtual void render() = 0;

		virtual void spriteRender(){};

		void setPosition(const XMVECTOR& _position)
		{ 
			m_position = _position; 
		}

		void setOrientation(const XMFLOAT3& _orientation){ m_orientation = _orientation; }

		void setScaling(const XMFLOAT3& _scaling){ m_scale = _scaling; }

		void setActor(snActor* _actor){ m_actor = _actor; }

		void setIsActive(bool _isActive){ m_isActive = _isActive; }

		bool getIsActive() const { return m_isActive; }

		//Return the list of components attached to this entity
		vector<IComponent*> const & getComponents() { return m_components; }

		//Return the position of the entity.
		const XMVECTOR& getPosition() const { return m_position; }

		//Add a component to the entity
		void addComponent(IComponent* _newComponent){ m_components.push_back(_newComponent); }

		void* operator new(size_t _count)
		{
			return _aligned_malloc(_count, 16);
		}

		void operator delete(void* _p)
		{
			_aligned_free(_p);
		}

		//Updat all the components of the entity
		void updateComponents()
		{
			for (vector<IComponent*>::iterator i = m_components.begin(); i != m_components.end(); ++i)
				(*i)->update();
		}

		void renderComponents()
		{
			for (vector<IComponent*>::iterator i = m_components.begin(); i != m_components.end(); ++i)
				(*i)->render();
		}
	};
}

#endif //I_WORLD_ENTITY_H
