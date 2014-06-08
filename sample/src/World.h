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
#ifndef WORLD_H
#define WORLD_H

#include <vector>

#include <DirectXMath.h>

#include "ComponentFloatingText.h"

namespace Supernova
{
	class snFixedConstraint;
	class snPointToPointConstraint;
	class snActorDynamic;
	class snIActor;
}

using namespace Supernova;
namespace Devil
{
	class IWorldEntity;
	class EntitySphere;
	class EntityBox;
	class EntityPlan;
	class EntityCollisionPoint;
	class EntityCamera;
	class EntityFixedConstraint;
	class EntityPointToPointConstraint;
	class WorldHUD;
	class EntityBoxLauncher;
	class EntityComposite;
	class EntitySkybox;

	class IComponent;

	class World
	{
	private:
		static World* m_Instance;

		std::vector<IWorldEntity*> m_EntityList;

		EntityCamera* m_camera;

		EntityCollisionPoint* m_collisionPoint;

		WorldHUD* m_hud;

		EntityComposite* m_monkeyBall;

	public:
		virtual ~World();

		static World* getInstance();
		static void shutdown();

		bool initialize();

		EntitySphere* createSphere(float _diameter, const XMVECTOR& _color);
		EntityBox* createBox(const XMFLOAT3&);
		EntityBox* createBox(const XMFLOAT3& _size, const XMFLOAT4& _color);
		EntityPlan* createPlan(const XMFLOAT2& _size, const XMFLOAT4& _color);
		EntityComposite* createComposite(snIActor* _actor, const XMFLOAT4& _color);
		EntityCamera* createCamera(const XMVECTOR& _position, const XMVECTOR& _lookAt, const XMVECTOR& _up);
		EntityFixedConstraint* createFixedConstraint(const snFixedConstraint* _constraint);
		EntityPointToPointConstraint* createPointToPointConstraint(const snPointToPointConstraint* _constraint);
		WorldHUD* createHUD();
		EntityBoxLauncher* createEntityBoxLauncher(unsigned int _count);
		EntityComposite* createMonkeyBall(snIActor* _actor, const XMFLOAT4& _color);
		EntitySkybox* createSkybox(IWorldEntity* _target, float _size, const XMFLOAT4& _color);

		//Delete all entities from the world.
		void clearWorld();

		void update(float _dt);
		void render();

		EntityCamera* getCamera() const;
		EntityComposite* getMonkeyBall() const;

		//Return the entity owner of the actor
		IWorldEntity* getEntityFromActor(snIActor* const _actor) const;

		void toggleCollisionPointActivation();
		void activateCollisionPoint();
		void deactivateCollisionPoint();

		void setPhysicsFPS(int _physicsFPS) const;
		void setGraphicsFPS(int _graphicsFPS) const;

	private:
		World();

		EntityCollisionPoint* createCollisionPoint(float _diameter);
	};

#define WORLD World::getInstance()
}

#endif