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

#ifndef SN_ICOLLIDER_H
#define SN_ICOLLIDER_H


#include "snVec.h"
#include "snGlobals.h"

namespace Supernova
{
	//forward declarations of derived types.
	class snCollisionResult;
	struct snAABB;
	class snMatrix44f;
	class snTransform;

	//Type enum to identify the kind of collider
	enum snEColliderType : unsigned char
	{
		snEColliderUnknown,
		snEColliderBox,
		snEColliderSphere,
		snEColliderCapsule,
		snEColliderHeightMap
	};

	class snICollider
	{

	protected:

		//The type of collider
		snEColliderType m_typeOfCollider;

	public:
		snICollider(){}

		virtual ~snICollider(){}

		//Initialize the collider. Should be called once all the parameters of the collider are set.
		virtual void initialize() = 0;

		//Move the collider using the given transform matrix.
		virtual void setTransform(const snTransform& _transform) = 0;

		//Compute the inertia tensor in a local frame.
		virtual void computeLocalInertiaTensor(float _mass, snMatrix44f& _inertiaTensor) const = 0;

		//Compute the bounding volume for this collider
		virtual void computeAABB(snAABB * const /*_boundingVolume*/) const = 0;

		//Return the feature the farthest along the normal.
		//_n : direction along which the search of the farthest feature has to be done.
		//_polygon : array of 4 vectors filled by the function containing the feature. 
		//_count : the number of vectors in the features : 1 = vertex, 2 = edge, 3 = triangle, 4 = face
		//_featureId : unique id of the feature.
		virtual void getClosestFeature(const snVec& _n, snVec* const _polygon, unsigned int& _count, unsigned int& _featureId) const
		{
			SN_UNREFERENCED_PARAMETER(_n);
			SN_UNREFERENCED_PARAMETER(_polygon);
			SN_UNREFERENCED_PARAMETER(_count);
			SN_UNREFERENCED_PARAMETER(_featureId);
			throw;
		};

		//Return the normal of a feature.
		virtual snVec getFeatureNormal(unsigned int _featureId) const 
		{ 
			//return Supernova::Vector::snVec4Set(0, 0, 0, 1); 
			SN_UNREFERENCED_PARAMETER(_featureId);
			throw;
		};

#pragma region GJK
		//////////////////////////////////////////////////////////////////////
		// GJK Interface													//
		//////////////////////////////////////////////////////////////////////

		//Return the farthest point in the direction provided as parameter.
		virtual snVec support(const snVec& _direction) const
		{
			SN_UNREFERENCED_PARAMETER(_direction);
			throw;
		}

		//Return any point making the collider
		virtual snVec anyPoint() const
		{
			throw;
		}

#pragma endregion

#pragma region SAT
		//////////////////////////////////////////////////////////////////////
		// SAT Interface													//
		//////////////////////////////////////////////////////////////////////

		//Fill in an array of unique normals to be tested by the SAT algorithms.
		//_arraySize is the size of the array.
		//Returns the number of normals available. Returns -1 if the array is not big enough.
		virtual int getUniqueNormals(snVec* _arrayNormals, int _arraySize) const
		{
			SN_UNREFERENCED_PARAMETER(_arrayNormals);
			SN_UNREFERENCED_PARAMETER(_arraySize);
			throw;
		}

		//Project the collider along an axis and return the min and max values.
		virtual void projectToAxis(const snVec& _axis, float& _min, float& _max) const
		{
			SN_UNREFERENCED_PARAMETER(_axis);
			SN_UNREFERENCED_PARAMETER(_min);
			SN_UNREFERENCED_PARAMETER(_max);
			throw;
		}

#pragma endregion

		snEColliderType getTypeOfCollider() const { return m_typeOfCollider; }

		void* operator new(size_t _count)
		{
			return _aligned_malloc(_count, SN_ALIGN_SIZE);
		}

		void operator delete(void* _p)
		{
			_aligned_free(_p);
		}
	};
}

#endif // SN_ICOLLIDER_H