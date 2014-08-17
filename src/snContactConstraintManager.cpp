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

#include "snContactConstraintManager.h"
#include "snContactConstraint.h"
#include "snRigidbody.h"

using namespace Supernova::Vector;

namespace Supernova
{
	snContactConstraintManager::snContactConstraintManager() : m_collisionConstraints(), m_currentConstraintId(0)
	{}

	snContactConstraintManager::~snContactConstraintManager()
	{
		for (vector<snContactConstraint*>::iterator i = m_collisionConstraints.begin(); i != m_collisionConstraints.end(); ++i)
		{
			if ((*i) == 0)
				continue;

			delete *i;
		}
		m_collisionConstraints.clear();
	}

	snContactConstraint* const snContactConstraintManager::getAvailableConstraint()
	{
		if (m_currentConstraintId < m_collisionConstraints.size())
		{
			snContactConstraint* cc = m_collisionConstraints[m_currentConstraintId];
			if(cc == 0)
			{
				cc = new snContactConstraint();
				m_collisionConstraints[m_currentConstraintId] = cc;
			}
			++m_currentConstraintId;
			return cc;
		}
		else
		{
			snContactConstraint* npConstraint = new snContactConstraint();
			m_collisionConstraints.push_back(npConstraint);
			++m_currentConstraintId;

			return npConstraint;
		}
	}

	void snContactConstraintManager::prepareActiveConstraint(float _dt)
	{
		for (vector<snContactConstraint*>::iterator constraint = m_collisionConstraints.begin(); constraint != m_collisionConstraints.end(); ++constraint)
		{
			if((*constraint) == 0)
			{
				continue;
			}

			if ((*constraint)->getIsActive())
			{
				(*constraint)->prepare(_dt);
			}
		}
		
	}

	void snContactConstraintManager::resolveActiveConstraints()
	{
		for (vector<snContactConstraint*>::iterator constraint = m_collisionConstraints.begin(); constraint != m_collisionConstraints.end(); ++constraint)
		{
			if ((*constraint) != 0 && (*constraint)->getIsActive())
			{
				(*constraint)->resolve();
			}
		}
	}

	void snContactConstraintManager::preBroadPhase()
	{
		m_currentConstraintId = 0;
	}

	void snContactConstraintManager::postBroadPhase()
	{
		//deactivate all unused constraints
		for (unsigned int i = m_currentConstraintId; i < m_collisionConstraints.size(); ++i)
		{
			snContactConstraint* cc = m_collisionConstraints[i];
			if(cc != 0)
			{
				cc->setIsActive(false);
			}
		}
	}

	void snContactConstraintManager::setSleepingBody(snRigidbody const * const _body)
	{
		for (vector<snContactConstraint*>::iterator constraint = m_collisionConstraints.begin(); constraint != m_collisionConstraints.end(); ++constraint)
		{
			if(*constraint == 0)
			{
				continue;
			}

			//Ignore inactive contraints
			if (!(*constraint)->getIsActive())
			{
				continue;
			}

			snRigidbody * const * const bodies = (*constraint)->getBodies();
			if(bodies[0] != _body && bodies[1] != _body) //Check if the rigidbody given in parameter is a part of the constraint.
			{
				continue;
			}

			//Check if this is a sleeping constraint
			if((bodies[0]->isStatic() && !bodies[1]->isAwake()) ||
				(bodies[1]->isStatic() && !bodies[0]->isAwake()) ||
				(!bodies[0]->isAwake() && !bodies[1]->isAwake())
				)
			{
				//Save the sleeping constraint.
				m_sleepingConstraint.push_back((*constraint));
				*constraint = 0;
			}
		}
	}

	void snContactConstraintManager::awakeConstraint(snRigidbody * const _body)
	{
		vector<snRigidbody*> toAwake;
		for(vector<snContactConstraint*>::iterator sc = m_sleepingConstraint.begin(); sc != m_sleepingConstraint.end(); ++sc)
		{
			if((*sc) == 0)
			{
				continue;
			}

			snRigidbody * const * const bodies = (*sc)->getBodies();

			//Check if the constraint contains the activated body
			snRigidbody* otherBody = 0;
			if(bodies[0] == _body)
			{
				otherBody = bodies[1];
			}
			else if(bodies[1] == _body)
			{
				otherBody = bodies[0];
			}
			else
			{
				continue;
			}

			//Move the constraint from the sleeping constraints to the activated constraints
			if (m_currentConstraintId < m_collisionConstraints.size())
			{
				m_collisionConstraints[m_currentConstraintId] = (*sc);
			}
			else
			{
				m_collisionConstraints.push_back(*sc);
			}
			++m_currentConstraintId;

			//Remove the constraint from the sleeping constraints
			*sc = 0;

			//save the other rigidbody to the list of bodies to awake if it's asleep and non static.
			if(!otherBody->isAwake() && !otherBody->isStatic())
			{
				toAwake.push_back(otherBody);
			}
		}

		//Awake other bodies
		for(vector<snRigidbody*>::iterator i = toAwake.begin(); i != toAwake.end(); ++i)
		{
			if((*i)->isAwake())
			{
				continue;
			}

			(*i)->setAwake(true);
			awakeConstraint(*i);
		}

		//Clear the null constraints in the sleeping list
		vector<snContactConstraint*>::iterator iter = m_sleepingConstraint.begin();
		while(iter != m_sleepingConstraint.end())
		{
			if(*iter == 0)
			{
				iter = m_sleepingConstraint.erase(iter);
			}
			else
			{
				++iter;
			}
		}
	}
}