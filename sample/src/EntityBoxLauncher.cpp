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

#include "EntityBoxLauncher.h"

#include "snFactory.h"
#include "snScene.h"
#include "snVector4f-inl.h"
#include "snActorDynamic.h"
#include "snColliderBox.h"

#include "World.h"
#include "EntityCamera.h"
#include "EntityBox.h"

#include "Input.h"

namespace Devil
{
	//Default constructor
	EntityBoxLauncher::EntityBoxLauncher() : m_boxes(), m_count(0){}

	//Default destructor
	EntityBoxLauncher::~EntityBoxLauncher(){}

	//Initialize the launcher.
	void EntityBoxLauncher::initialize(unsigned int _count)
	{
		m_count = _count;

		for (unsigned int i = 0; i < _count; ++i)
		{
			float width = 5;
			float height = 5;
			float depth = 5;
			
			//create actor
			snActorDynamic* act = 0;
			int actorId = -1;

			snScene* myScene = SUPERNOVA->getScene(0);
			myScene->createActorDynamic(&act, actorId);

			act->setName("projectile");
			
			act->getPhysicMaterial().m_restitution = 1;
			act->getPhysicMaterial().m_friction = 0;

			//create collider
			snColliderBox* collider = new snColliderBox();
			collider->setSize(snVector4f(width, height, depth, 0));
			act->addCollider(collider);
			act->updateMassAndInertia(50);
			act->initialize();

			EntityBox* box = WORLD->createBox(XMFLOAT3(width, height, depth), XMFLOAT4(0.8f, 1, 1, 1));
			box->setActor(act);

			//deactive the entity
			box->setIsActive(false);
			act->setIsActive(false);

			//store a pointer to the entity
			m_boxes.push_back(box);

		}
	}

	void EntityBoxLauncher::update()
	{
		if (!INPUT->isKeyDown(' '))
			return;

		INPUT->keyUp(' ');

		//get the box
		EntityBox* box = m_boxes[0];
		snActorDynamic* act = static_cast<snActorDynamic*>(box->getActor());

		//set its position
		snVector4f pos(WORLD->getCamera()->getPosition());
		act->setPosition(pos);

		//set its linear velocity
		snVector4f linVel = WORLD->getCamera()->getLookAt() - WORLD->getCamera()->getPosition();
		linVel.normalize();
		linVel = linVel * 300;
		linVel.setW(0);
		act->setLinearVelocity(linVel);

		//set its orientation
		act->setOrientation(snVector4f(0, 0, 0, 1));

		//set its angular velocity
		act->setAngularVelocity(snVector4f(0, 0, 0, 0));
		act->initialize();

		box->setIsActive(true);
		act->setIsActive(true);
	}

	void EntityBoxLauncher::render(){}
}