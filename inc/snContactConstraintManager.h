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

//#include "snIConstraint.h"

#include <vector>
using std::vector;

#include <mutex>
using std::mutex;

namespace Supernova
{
	class snIConstraint;
	class snContactConstraint;
	class snFrictionConstraint;

	//Store and manage the contact constraints used in a scene.
	class snContactConstraintManager
	{
	private:
		//List of constraints created by the collision detection system.
		vector<snIConstraint*> m_collisionConstraints;

		//Id of the next available constraint
		unsigned int m_currentConstraintId;

		//Protect access to the list of constraints
		mutex m_protect;

	public:
		//Default constructor
		snContactConstraintManager();

		//Destructor. Clean all the allocated constraints
		~snContactConstraintManager();

		//Return pointers to available contact and friction constraints.
		void getAvailableConstraints(snContactConstraint** _contact, snFrictionConstraint** _friction);

		//Prepare all the currently active constraints.
		void prepareActiveConstraint(float _dt);

		//Solve all the currently active constraints.
		void resolveActiveConstraints();

		//Prepare the manager for the broad phase step.
		void preBroadPhase();

		//Clean the manager after the broad phase step.
		void postBroadPhase();

	private:
		//Return an available constraint. If no available constraint is found, it will be created.
		//This function is private and must not be called from the outside for thread safety reasons.
		snIConstraint* getAvailableConstraint();
	};
}

#endif //ifndef SN_CONTACT_CONSTRAINT_MANAGER_H