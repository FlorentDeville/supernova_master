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

#ifndef SN_SWEEP_MANAGER_H
#define SN_SWEEP_MANAGER_H

#include <list>
using std::list;

#include "snVec.h"

#include <vector>
using std::vector;

#include "snGlobals.h"

namespace Supernova
{
	class snIActor;
	class snScene;
	struct snAABB;

	typedef void(snScene::*snPairFoundCallback)(snIActor* _a, snIActor* _b);

	//Manager to handle and execute the sweep and prune broad phase algorithm.
	class SN_ALIGN snSweepManager
	{
	private:
		//List of actor sorted using their aabb
		list<snIActor*> m_sortedActors;

		//Axis to use to sort the actors
		unsigned char m_axis;

		//Axis to use to sort the actors for the next iteration
		unsigned char m_deferredAxis;

		//fnuction to call when a pair that might be colliding is found
		snPairFoundCallback m_callback;

		//Scene object
		snScene* m_scene;

		//sum of AABB's centers used to compute the variance.
		snVec m_sum;

		//Squared sum of AABB's centers used to compute the variance.
		snVec m_squaredSum;

#ifdef SN_DEBUGGER
		//Number of pair found 
		unsigned int m_collisionPairFound;
#endif //ifdef SN_DEBUGGER
	public:
		//Default constructor
		snSweepManager();

		//Default destructor
		~snSweepManager();

		//Clear the list of actors
		void clearList();

		//Set the callback function to call when a possibly colliding pair of actors is found.
		void setCallback(snScene* _scene, snPairFoundCallback _callback);

		//Add the actor to the list of actors to use in the sweep and prune process.
		void addActor(snIActor* _actor);

		//Remove an actor from the list of actor to use in the sweep and prune.
		// _actor : pointer to the actor to delete.
		void removeActor(snIActor const * const _actor);

		//Sort the list using actor's aabbs.
		void preBroadPhase();

		//Check for possibily colliding pair of actors and call the callback function everytime a pair is found.
		void broadPhase();

		//Compute which axis to use to sort the actors.
		void postBroadPhase();

		//For a given aabb, use the seep list to find all the possibly colliding actors
		void getPossiblyCollidingActor(const snAABB& _aabb, vector<snIActor*>& _pca) const;
	};
}
#endif //ifndef SN_SWEEP_MANAGER_H