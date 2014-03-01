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
#include "snGlobals.h"

namespace Supernova
{
	/*Forward declaration*/
	class snVector4f;
	class snCollisionResult;
	class snICollider;
	class snColliderBox;
	class snColliderSphere;
	class snColliderPlan;
	class snActor;

	class SN_ALIGN snCollision
	{
	
	public:
		snCollision();
		virtual ~snCollision();

		static snCollisionResult queryTestCollision(snActor*, snActor*);

		static snCollisionResult queryTestCollision(const snColliderBox&, const snColliderBox&);
		static snCollisionResult queryTestCollision(const snColliderSphere&, const snColliderSphere&);
		static snCollisionResult queryTestCollision(const snColliderBox&, const snColliderSphere&);

		static snCollisionResult queryTestCollision(const snColliderBox&, const snColliderPlan&);
		static snCollisionResult queryTestCollision(const snColliderSphere&, const snColliderPlan&);

		void* operator new(size_t _count)
		{
			return _aligned_malloc(_count, 16);
		}

		void operator delete(void* _p)
		{
			_aligned_free(_p);
		}

	private:
		static void computeProjection(const snVector4f& _axis, const snVector4f* _vertices, int _verticesCount, float& _min, float& max);
		static bool computeOverlap(const snVector4f& _axis, const snVector4f* _bodyAVertices, const snVector4f* _bodyBVertices, int _vertexCount, snVector4f& _separatingAxis, float& _overlap);
	};
}

#endif //SN_COLLISION_H