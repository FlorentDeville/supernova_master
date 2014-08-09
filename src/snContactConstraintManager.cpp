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
			return m_collisionConstraints[m_currentConstraintId++];
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
			if ((*constraint)->getIsActive())
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
			m_collisionConstraints[i]->setIsActive(false);
	}
}