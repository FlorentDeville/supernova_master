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

#ifndef SN_ARBITER_H
#define SN_ARBITER_H

#include "snContactConstraint.h"

namespace Supernova
{
	class snArbiter
	{
	public:
		static const int MAX_CONTACT = 4;

		unsigned int m_contactCount;

		snContactConstraint* m_constraints[MAX_CONTACT];

		snRigidbody* m_body1;

		snRigidbody* m_body2;

		bool m_dirty;

	public:
		snArbiter(snRigidbody* _body1, snRigidbody* _body2) 
			: m_contactCount(0)
			, m_body1(_body1)
			, m_body2(_body2)
			, m_dirty(true)
		{
			for(unsigned int i = 0; i < MAX_CONTACT; ++i)
				m_constraints[i] = nullptr;
		}

		~snArbiter()
		{
			for(snContactConstraint* constraint : m_constraints)
			{
				if(constraint != nullptr)
					delete constraint;
			}
		}

		void update(const snCollisionResult& _result, snContactConstraintManager& _manager)
		{
			snContactConstraint* mergedConstraints[MAX_CONTACT];
			unsigned int mergedConstraintsCount = 0;

			//Loop through each new contacts
			for(unsigned int i = 0; i < _result.m_contactsCount; ++i)
			{
				const snContact& newContact = _result.m_contacts[i];

				snContactConstraint* foundConstraint = nullptr;

				//Look for this contact
				for(unsigned int k = 0; k < m_contactCount; ++k)
				{
					snContactConstraint* constraint = m_constraints[k];
					if(constraint == nullptr)
						continue;

					if(constraint->m_featuresId[0] == newContact.m_featuresId[0] 
					&& constraint->m_featuresId[1] == newContact.m_featuresId[1])
					{
						foundConstraint = constraint;
						m_constraints[k] = nullptr;
						break;
					}
				}

				//Constraint not found, use an empty one
				if(foundConstraint == nullptr)
				{
					foundConstraint = _manager.getConstraint();
					foundConstraint->initialize(m_body1, m_body2, newContact.m_normal, newContact.m_point, newContact.m_penetration,
						_manager.getScene());
					foundConstraint->m_featuresId[0] = newContact.m_featuresId[0];
					foundConstraint->m_featuresId[1] = newContact.m_featuresId[1];
				}
				else //Update and save the constraint
				{
					foundConstraint->update(newContact.m_normal, newContact.m_point, newContact.m_penetration);
				}
				mergedConstraints[mergedConstraintsCount++] = foundConstraint;
			}

			//Set the merged constraints to the arbiter.
			for(unsigned int i = 0; i < mergedConstraintsCount; ++i)
			{
				if(m_constraints[i] != nullptr)
					_manager.pushConstraint(m_constraints[i]);
				
				m_constraints[i] = mergedConstraints[i];
				m_constraints[i]->setIsActive(true);
			}

			for(unsigned int i = mergedConstraintsCount; i < MAX_CONTACT; ++i)
			{
				if(m_constraints[i] != nullptr)
				{
					_manager.pushConstraint(m_constraints[i]);
					m_constraints[i] = nullptr;
				}
			}

			m_contactCount = mergedConstraintsCount;
		}

	};
}

#endif //ifndef SN_ARBITER_H