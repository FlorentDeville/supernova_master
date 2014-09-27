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
#include "snCollisionResult.h"
#include "snArbiter.h"

using namespace Supernova::Vector;

namespace Supernova
{
	snContactConstraintManager::snContactConstraintManager(const snScene* _scene) 
		: m_activeConstraints()
		, m_isSleepingStateAuthorized(true)
		, m_scene(_scene)
	{}

	snContactConstraintManager::~snContactConstraintManager()
	{
		for(auto& pair : m_activeConstraints)
		{
			if ((pair.second) == 0)
				continue;

			delete pair.second;
		}

		m_activeConstraints.clear();
		
		while(!m_constraintsPool.empty())
		{
			snContactConstraint* constraint = m_constraintsPool.top();
			if(constraint != 0)
				delete constraint;

			m_constraintsPool.pop();
		}
	}

	void snContactConstraintManager::addOrUpdateContact(snRigidbody* _body1, snRigidbody* _body2, const snCollisionResult& _contact)
	{
		//look for the key
		snArbiterKey key = MAKE_ARBITER_KEY(_body1->getId(), _body2->getId());
		map<snArbiterKey, snArbiter*>::iterator i = m_activeConstraints.find(key);

		if(i != m_activeConstraints.end())
		{
			i->second->update(_body1, _body2, _contact, *this);
			i->second->m_dirty = false;
		}
		else
		{
			snArbiter* newArbiter = new snArbiter(_body1, _body2);
			for(unsigned int i = 0; i < snArbiter::MAX_CONTACT; ++i)
			{
				snContactConstraint* newConstraint = getConstraint();
				
				if(i < _contact.m_contactsCount)
				{
					const snContact& newContact = _contact.m_contacts[i];
					newConstraint->initialize(_body1, _body2, newContact.m_normal, newContact.m_point, newContact.m_penetration, m_scene);
					newConstraint->m_featuresId[0] = newContact.m_featuresId[0];
					newConstraint->m_featuresId[1] = newContact.m_featuresId[1];
				}

				newArbiter->m_constraints[i] = newConstraint;
			}
			newArbiter->m_contactCount = _contact.m_contactsCount;
			newArbiter->m_dirty = false;
			m_activeConstraints[key] = newArbiter;
		}
	}

	void snContactConstraintManager::prepareActiveConstraint(float _dt)
	{
		for(auto& pair : m_activeConstraints)
		{
			if(pair.second == 0)
				continue;

			snArbiter* arbiter = pair.second;
			for(unsigned int i = 0; i < arbiter->m_contactCount; ++i)
			{
				if(arbiter->m_constraints[i] != nullptr && arbiter->m_constraints[i]->getIsActive())
					arbiter->m_constraints[i]->prepare(_dt);
			}
		}		
	}

	void snContactConstraintManager::resolveActiveConstraints()
	{
		for(auto& pair : m_activeConstraints)
		{
			snArbiter* arbiter = pair.second;
			if (arbiter == 0)
				continue;

			for(unsigned int i = 0; i < arbiter->m_contactCount; ++i)
			{
				if(arbiter->m_constraints[i] != nullptr && arbiter->m_constraints[i]->getIsActive())
					arbiter->m_constraints[i]->resolve();
			}
		}
	}

	void snContactConstraintManager::preBroadPhase()
	{
		for(std::pair<snArbiterKey, snArbiter*> arbiter : m_activeConstraints)
			arbiter.second->m_dirty = true;
	}

	void snContactConstraintManager::postBroadPhase()
	{
		//remove all dirty arbiter
		map<snArbiterKey, snArbiter*>::iterator iter = m_activeConstraints.begin();

		while(iter != m_activeConstraints.end())
		{
			if(iter->second->m_dirty)
			{
				snArbiter* arbiter = iter->second;
				for(unsigned int i = 0; i < arbiter->MAX_CONTACT; ++i)
				{
					if(arbiter->m_constraints[i] != nullptr)
					{
						m_constraintsPool.push(arbiter->m_constraints[i]);
						arbiter->m_constraints[i] = nullptr;
					}
				}

				delete arbiter;
				iter = m_activeConstraints.erase(iter);
			}
			else
				++iter;
		}
	}

	void snContactConstraintManager::setSleepingBody(snRigidbody const * const _body)
	{
		////Check if the body exists in the sleeping structure.
		//map<snObjectId, snConstraintsPtrArray>::iterator idExists = m_sleepingGraph.find(_body->m_id);
		//if(idExists == m_sleepingGraph.end())
		//{
		//	m_sleepingGraph.insert(snConstraintGraphElement(_body->m_id, snConstraintsPtrArray()));
		//}

		////Loop through each constraints to find the ones to put asleep
		//for(auto& pair : m_activeConstraints)
		//{
		//	if(pair.second == 0 || !pair.second->getIsActive())
		//	{
		//		continue;
		//	}

		//	snRigidbody * const * const bodies = pair.second->getBodies();
		//	snRigidbody const * otherBody = 0;
		//	if(bodies[0] == _body ) //Check if the rigidbody given in parameter is a part of the constraint.
		//	{
		//		otherBody = bodies[1];
		//	}
		//	else if(bodies[1] == _body)
		//	{
		//		otherBody = bodies[0];
		//	}
		//	else
		//	{
		//		continue;
		//	}

		//	//Check if the other body exists in the graph of sleeping bodies
		//	map<snObjectId, snConstraintsPtrArray>::iterator idExists = m_sleepingGraph.find(otherBody->m_id);
		//	if(idExists == m_sleepingGraph.end())
		//	{
		//		m_sleepingGraph.insert(snConstraintGraphElement(otherBody->m_id, snConstraintsPtrArray()));
		//	}

		//	//If the other body is sleeping or static, set the constraint to sleep.
		//	if(otherBody->isStatic() || !otherBody->isAwake())
		//	{
		//		pair.second->setAwake(false);
		//		m_sleepingGraph[otherBody->m_id].push_back(pair.second);
		//		m_sleepingGraph[_body->m_id].push_back(pair.second);
		//		pair.second = 0;
		//	}
		//}
	}

	void snContactConstraintManager::awakeConstraint(snRigidbody * const _body)
	{
		//snConstraintsPtrArray& constraints = m_sleepingGraph[_body->m_id];
		//if(constraints.size() == 0)
		//	return;

		////Loop through each constraints for the body
		//for(vector<snContactConstraint*>::iterator i = constraints.begin(); i != constraints.end(); ++i)
		//{
		//	//Get the other body involved in the constraint
		//	snRigidbody * const * const bodies = (*i)->getBodies();
		//	snRigidbody* otherBody = 0;
		//	if(bodies[0] == _body)
		//	{
		//		otherBody = bodies[1];
		//	}
		//	else
		//	{
		//		otherBody = bodies[0];
		//	}

		//	//Don't do anything if the other body is already awake. It means
		//	//we are in a recursive call and the constraint was already awaken earlier.
		//	if(otherBody->isAwake())
		//	{
		//		continue;
		//	}

		//	if((*i)->getIsAwake())
		//	{
		//		continue;
		//	}

		//	//Move the constraint from the sleeping constraints to the activated constraints
		//	(*i)->setAwake(true);
		//	/*if (m_currentConstraintId < m_collisionConstraints.size())
		//	{
		//		m_collisionConstraints[m_currentConstraintId] = (*i);
		//	}
		//	else
		//	{
		//		m_collisionConstraints.push_back(*i);
		//	}
		//	++m_currentConstraintId;*/

		//	//recursive call to activate the other body.
		//	otherBody->setAwake(true);
		//	awakeConstraint(otherBody);
		//}

		////Clear the list of constraints
		//constraints.clear();
	}

	void snContactConstraintManager::setIsSleepingStateAuthorized(bool _isSleepingStateAuthorized)
	{
		m_isSleepingStateAuthorized = _isSleepingStateAuthorized;
	}

	snContactConstraint* const snContactConstraintManager::getConstraint()
	{
		if(m_constraintsPool.empty())
		{
			return new snContactConstraint();
		}
		else
		{
			snContactConstraint* constraint = m_constraintsPool.top();
			m_constraintsPool.pop();
			return constraint;
		}
	}

	void snContactConstraintManager::pushConstraint(snContactConstraint* _constraint)
	{
		m_constraintsPool.push(_constraint);
	}

	const snScene* snContactConstraintManager::getScene() const
	{
		return m_scene;
	}
}