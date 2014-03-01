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

	public:
		IWorldEntity() : m_isActive(true)
		{
			m_position = XMVectorSet(0, 0, 0, 1);
			m_orientation = XMFLOAT3(0, 0, 0);
			m_scale = XMFLOAT3(1, 1, 1);
		};

		virtual ~IWorldEntity(){};
		virtual void update() = 0;
		virtual void render() = 0;

		void setPosition(const XMVECTOR& _position)
		{ 
			m_position = _position; 
		}

		void setOrientation(const XMFLOAT3& _orientation){ m_orientation = _orientation; }
		void setScaling(const XMFLOAT3& _scaling){ m_scale = _scaling; }

		void setActor(snActor* _actor){ m_actor = _actor; }

		void setIsActive(bool _isActive){ m_isActive = _isActive; }

		bool getIsActive() const { return m_isActive; }

		//Return the position of the entity.
		const XMVECTOR& getPosition() const { return m_position; }
	};
}

#endif //I_WORLD_ENTITY_H
