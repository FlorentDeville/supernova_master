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

#include "snWorld.h"
#include "snScene.h"
#include "snVec.h"
#include "snRigidbody.h"
#include "snOBB.h"

#include "World.h"
#include "EntityCamera.h"
#include "EntityBox.h"

#include "Input.h"

using namespace Supernova::Vector;

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
			snhScene myScene = WORLD->getPhysicsScene();
			snhRigidbody act = SUPERNOVA->registerObject(new snRigidbody());
			myScene->attachActor(act);

			act->setName("projectile");
			
			act->getPhysicMaterial().m_restitution = 1;
			act->getPhysicMaterial().m_friction = 0;

			//create collider
			snOBB* collider = new snOBB(Supernova::Vector::snVec4Set(width, height, depth, 0) * 0.5f);
			act->addCollider(collider);
			act->updateMassAndInertia(50);
			act->initializeDynamic();

			EntityBox* box = WORLD->createBox(XMFLOAT3(width, height, depth), XMFLOAT4(0.8f, 1, 1, 1));
			box->setActor(act.getPtr());

			//deactive the entity
			box->setIsActive(false);
			act->setIsActive(false);

			//store a pointer to the entity
			m_boxes.push_back(box);

		}
	}

	void EntityBoxLauncher::update()
	{
		if (INPUT->getMessage(Devil::Input::InputMessage::SHOOT) != 1)
			return;

		//get the box
		EntityBox* box = m_boxes[0];
		snRigidbody* act = box->getActor();

		//set its position
		snVec pos;
		pos = WORLD->getCamera()->getPosition();
		act->setPosition(pos);

		//set its linear velocity
		snVec linVel;
		linVel = WORLD->getCamera()->getLookAt() - WORLD->getCamera()->getPosition();
		Supernova::Vector::snVec3Normalize(linVel);
		linVel = linVel * 300;
		Supernova::Vector::snVec4SetW(linVel, 0);
		act->setLinearVelocity(linVel);

		//set its orientation
		act->setOrientation(Supernova::Vector::snVec4Set(0, 0, 0, 1));

		//set its angular velocity
		act->setAngularVelocity(Supernova::Vector::snVec4Set(0, 0, 0, 0));
		act->initializeDynamic();

		box->setIsActive(true);
		act->setIsActive(true);
	}

	void EntityBoxLauncher::render(){}
}