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

#include "snAABB.h"

using namespace Supernova::Vector;

namespace Supernova
{
	bool AABBOverlap(snAABB const * const _a, snAABB const * const _b)
	{
		//if a->max >= b->min && a->min <= b->max 
		//	return true
		//else 
		//	return false

		__m128 compare = _mm_cmpge_ps(_a->m_max, _b->m_min);
		__m128i iCompare = _mm_castps_si128(compare);
		int resCompare1 = _mm_movemask_epi8(iCompare);

		compare = _mm_cmple_ps(_a->m_min, _b->m_max);
		iCompare = _mm_castps_si128(compare);
		int resCompare2 = _mm_movemask_epi8(iCompare);

		int resCompare = resCompare1 & resCompare2;

		return (resCompare & 0x0fff) == 0x0fff;
	}

	/// <summary>
	/// Merges two AABBs into one.
	/// </summary>
	/// <param name="_first">The first AABB to merge.</param>
	/// <param name="_second">The second AABB to merge.</param>
	/// <param name="_merge">The result AABB.</param>
	void mergeAABB(const snAABB& _first, const snAABB& _second, snAABB& _merge)
	{
		//compare the maximum
		__m128 compare = _mm_cmpge_ps(_first.m_max, _second.m_max);

		//compare contains 0x0000 where first is the smallest and 0xFFFF where first is the biggest. So the merge.max = (first & compare) | (second & ~compare)
		__m128 firstHalf = _mm_and_ps(_first.m_max, compare);
		__m128 secondHalf = _mm_andnot_ps(compare, _second.m_max);
		_merge.m_max = _mm_or_ps(firstHalf, secondHalf);

		//compare the minimum
		compare = _mm_cmple_ps(_first.m_min, _second.m_min);

		//compare contains 0x0000 where first is the biggest and 0xFFFF where first is the smallest. So the merge.min = (first & compare) | (second & ~compare)
		firstHalf = _mm_and_ps(_first.m_min, compare);
		secondHalf = _mm_andnot_ps(compare, _second.m_min);
		_merge.m_min = _mm_or_ps(firstHalf, secondHalf);
	}

	bool isInside(const snAABB& _bb, const snVec& _p)
	{
		if(snVec3Inferior(_p, _bb.m_min))
			return false;

		if(snVec3Inferior(_bb.m_max, _p))
			return false;

		return true;
	}
}