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

#ifndef SN_COLLISION_H
#define SN_COLLISION_H

#include <malloc.h>
#include <map>
using std::map;

#include "snGlobals.h"
#include "snCollisionResult.h"
#include "snICollider.h"

//Compute the key to retrieve the collision query function in the map. The parameters are the type of colliders.
#define SN_COLLISION_KEY(a, b) ((a<<8) | b)

namespace Supernova
{
	/*Forward declaration*/
	class snColliderBox;
	class snColliderSphere;
	class snColliderPlan;
	class snIActor;

	//Make collision tests between all the different types of colliders available.
	class SN_ALIGN snCollision
	{
	
	private:
		//Typedef of pointers to the collision query functions.
		typedef snCollisionResult(*snQueryTestCollisionFunction)(const snICollider* const, const snICollider* const);

		typedef std::pair<unsigned short, snQueryTestCollisionFunction> snCollisionQueryMapElement;

		typedef map<unsigned short, snQueryTestCollisionFunction> snCollisionQueryMap;

		//Map between a key and a collision query function.
		snCollisionQueryMap m_collisionQueryMap;

	public:
		snCollision();
		virtual ~snCollision();

		void queryTestCollision(snIActor*, snIActor*, snCollisionResult* _results, unsigned int _maxResultCount, unsigned int* _resultsCount) const;

		void* operator new(size_t _count)
		{
			return _aligned_malloc(_count, 16);
		}

		void operator delete(void* _p)
		{
			_aligned_free(_p);
		}

	private:

		snCollisionResult invokeQueryTestCollision(const snICollider* const _c1, const snICollider* const _c2) const;

		//Check collision between two boxes
		static snCollisionResult queryTestCollisionBoxVersusBox(const snICollider* const _c1, const snICollider* const _c2);

		static snCollisionResult queryTestCollisionSphereVersusSphere(const snICollider* const _c1, const snICollider* const _c2);

		static snCollisionResult queryTestCollisionBoxVersusSphere(const snICollider* const _c1, const snICollider* const _c2);

		static snCollisionResult queryTestCollisionBoxVersusPlan(const snICollider* const _c1, const snICollider* const _c2);

		static snCollisionResult queryTestCollisionSphereVersusPlan(const snICollider* const _c1, const snICollider* const _c2);

		static bool computeBoxesOverlap(const snColliderBox& _c1, const snColliderBox& _c2, const snVec& _axis, snVec& _separatingAxis, float& _overlap);
	};
}

#endif //SN_COLLISION_H