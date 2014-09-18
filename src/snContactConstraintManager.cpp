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
	snContactConstraintManager::snContactConstraintManager() : 
		m_collisionConstraints(), 
		m_currentConstraintId(0),
		m_isSleepingStateAuthorized(true)
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
		//Check if the body exists in the sleeping structure.
		map<snObjectId, snConstraintsPtrArray>::iterator idExists = m_sleepingGraph.find(_body->m_id);
		if(idExists == m_sleepingGraph.end())
		{
			m_sleepingGraph.insert(snConstraintGraphElement(_body->m_id, snConstraintsPtrArray()));
		}

		//Loop through each constraints to find the ones to put asleep
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
			snRigidbody const * otherBody = 0;
			if(bodies[0] == _body ) //Check if the rigidbody given in parameter is a part of the constraint.
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

			//Check if the other body exists in the graph of sleeping bodies
			map<snObjectId, snConstraintsPtrArray>::iterator idExists = m_sleepingGraph.find(otherBody->m_id);
			if(idExists == m_sleepingGraph.end())
			{
				m_sleepingGraph.insert(snConstraintGraphElement(otherBody->m_id, snConstraintsPtrArray()));
			}

			//If the other body is sleeping or static, set the constraint to sleep.
			if(otherBody->isStatic() || !otherBody->isAwake())
			{
				(*constraint)->setAwake(false);
				m_sleepingGraph[otherBody->m_id].push_back(*constraint);
				m_sleepingGraph[_body->m_id].push_back(*constraint);
				*constraint = 0;
			}
		}
	}

	void snContactConstraintManager::awakeConstraint(snRigidbody * const _body)
	{
		snConstraintsPtrArray& constraints = m_sleepingGraph[_body->m_id];
		if(constraints.size() == 0)
			return;

		//Loop through each constraints for the body
		for(vector<snContactConstraint*>::iterator i = constraints.begin(); i != constraints.end(); ++i)
		{
			//Get the other body involved in the constraint
			snRigidbody * const * const bodies = (*i)->getBodies();
			snRigidbody* otherBody = 0;
			if(bodies[0] == _body)
			{
				otherBody = bodies[1];
			}
			else
			{
				otherBody = bodies[0];
			}

			//Don't do anything if the other body is already awake. It means
			//we are in a recursive call and the constraint was already awaken earlier.
			if(otherBody->isAwake())
			{
				continue;
			}

			if((*i)->getIsAwake())
			{
				continue;
			}

			//Move the constraint from the sleeping constraints to the activated constraints
			(*i)->setAwake(true);
			if (m_currentConstraintId < m_collisionConstraints.size())
			{
				m_collisionConstraints[m_currentConstraintId] = (*i);
			}
			else
			{
				m_collisionConstraints.push_back(*i);
			}
			++m_currentConstraintId;

			//recursive call to activate the other body.
			otherBody->setAwake(true);
			awakeConstraint(otherBody);
		}

		//Clear the list of constraints
		constraints.clear();
	}

	void snContactConstraintManager::setIsSleepingStateAuthorized(bool _isSleepingStateAuthorized)
	{
		m_isSleepingStateAuthorized = _isSleepingStateAuthorized;
	}
}