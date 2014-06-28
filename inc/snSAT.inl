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

#include "snSAT.h"
#include "snISATCollider.h"

#include "snVec.inl"
#include "snMath.h"
using namespace Supernova::Vector;

namespace Supernova
{
	template <class T, class U>
	bool snSAT::queryIntersection(const T& _c1, const U& _c2, snVec& _collisionNormal)
	{
		const int NORMAL_COUNT = 3;
		snVec s1Normals[NORMAL_COUNT];
		snVec s2Normals[NORMAL_COUNT];

		_c1.getUniqueNormals(s1Normals, NORMAL_COUNT);
		_c2.getUniqueNormals(s2Normals, NORMAL_COUNT);

		float smallestOverlap = SN_FLOAT_MAX;

		//compute collider overlap using the normals
		for (int i = 0; i < NORMAL_COUNT; ++i)
		{
			if (!queryOverlap<T, U>(_c1, _c2, s1Normals[i], _collisionNormal, smallestOverlap))
				return false;
			if (!queryOverlap<T, U>(_c1, _c2, s2Normals[i], _collisionNormal, smallestOverlap))
				return false;
		}

		//compute overlap for the cross product of the normals
		for (int i = 0; i < NORMAL_COUNT; ++i)
		{
			for (int j = i; j < NORMAL_COUNT; ++j)
			{
				snVec cross = snVec3Cross(s1Normals[i], s2Normals[j]);
				if (snVec3SquaredNorme(cross) != 1.f) continue;
				if (!queryOverlap<T, U>(_c1, _c2, cross, _collisionNormal, smallestOverlap))
					return false;
			}
		}

		return true;
	}

	template <class T, class U>
	static bool snSAT::queryOverlap(const T& _c1, const U& _c2, const snVec& _axis, snVec& _separatingAxis, float& _overlap)
	{
		float minS1, minS2, maxS1, maxS2;

		_c1.projectToAxis(_axis, minS2, maxS2);
		_c2.projectToAxis(_axis, minS1, maxS1);

		float diff1 = maxS1 - minS2;
		float diff2 = maxS2 - minS1;

		//no collision
		if (diff1 < 0.f || diff2 < 0.f)
			return false;

		//collision, get the minimum overlap with the axis
		if (diff1 < _overlap)
		{
			_overlap = diff1;
			_separatingAxis = _axis;
		}
		if (diff2 < _overlap)
		{
			_overlap = diff2;
			_separatingAxis = _axis * -1;
		}
		return true;
	}
}