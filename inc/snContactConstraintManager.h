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

#ifndef SN_CONTACT_CONSTRAINT_MANAGER_H
#define SN_CONTACT_CONSTRAINT_MANAGER_H


#include <vector>
using std::vector;

#include <mutex>
using std::mutex;

#include <map>
using std::map;

#include "snObject.h"

namespace Supernova
{
	class snContactConstraint;
	class snRigidbody;

	//Store and manage the contact constraints used in a scene.
	class snContactConstraintManager
	{
	private:
		//List of constraints created by the collision detection system.
		vector<snContactConstraint*> m_collisionConstraints;

		//Id of the next available constraint
		unsigned int m_currentConstraintId;

		//Flag to indicate if we can set bodies into sleeping state
		bool m_isSleepingStateAuthorized;

		//////////////////////////////////////////////////////////////////////
		//				Define the sleeping constraint graph				//
		//////////////////////////////////////////////////////////////////////

		//A vector of constraint pointers.
		typedef vector<snContactConstraint*> snConstraintsPtrArray;

		//A graph of constraints. The key is the id of the rigidbody. The values are the constraints of the body.
		typedef map<snObjectId, snConstraintsPtrArray> snConstraintGraph;

		//Single element of a constraint graph.
		typedef std::pair<snObjectId, snConstraintsPtrArray> snConstraintGraphElement;

		//Graph of sleeping constraints. 
		map<snObjectId, snConstraintsPtrArray> m_sleepingGraph;

	public:
		//Default constructor
		snContactConstraintManager();

		//Destructor. Clean all the allocated constraints
		~snContactConstraintManager();

		//Return an available constraint. If no available constraint is found, it will be created.
		// remarks : NOT THREAD SAFE!!!!
		snContactConstraint* const getAvailableConstraint();

		//Prepare all the currently active constraints.
		void prepareActiveConstraint(float _dt);

		//Solve all the currently active constraints.
		void resolveActiveConstraints();

		//Prepare the manager for the broad phase step.
		void preBroadPhase();

		//Clean the manager after the broad phase step.
		void postBroadPhase();

		//Save the constraint for the body _body to the sleeping constraints list.
		void setSleepingBody(snRigidbody const * const _body);

		//Awake the constraint for a a rigidbody and recursively the other rigidbody of the constraints.
		void awakeConstraint(snRigidbody * const _body);

		//Set the flag authorizing the sleeping state.
		// _isSleepingStateAuthorized : True to authorized sleeping state. False otherwise.
		void setIsSleepingStateAuthorized(bool _isSleepingStateAuthorized);
	private:
		
	};
}

#endif //ifndef SN_CONTACT_CONSTRAINT_MANAGER_H