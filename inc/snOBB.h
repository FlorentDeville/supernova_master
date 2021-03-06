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

#ifndef SN_OBB_H
#define SN_OBB_H

#include "snICollider.h"

namespace Supernova
{
	class snOBB : public snICollider
	{
	private:
		static const int VERTEX_COUNT = 8;
		static const int INDEX_COUNT = 24;

		//Position of the center of the obb.
		snVec m_pos;

		//Half size of the box
		snVec m_extends;

		//Normals expressed in wolrd coordinates.
		snVec m_normals[3];

		//The rest is here for cache purpose and needs to be cleaned.
		//Vertices composing the box.
		snVec m_box[VERTEX_COUNT];

		/*List of indices of vertices making each face of the box. Widing is anticlockwise.*/
		int m_idFaces[INDEX_COUNT];

		//Vertices composing the box in world coordinates.
		snVec m_worldBox[VERTEX_COUNT];

	public:
		//Constructor.
		snOBB(const snVec& _extends);

		//Destructor.
		~snOBB();

		snVec getPosition() const;

		//Return a vector containing half the size of each dimension.
		snVec getExtends() const;

		//Initialize the collider. Should be called once all the parameters of the collider are set.
		void initialize();

		//Update the collider using the transform.
		void updateFromTransform();

		//Compute the inertia tensor in a local frame.
		void computeLocalInertiaTensor(float _mass, snMatrix44f& _inertiaTensor) const;

		/*Calculate the closest point to the box of the point given in parameter.*/
		snVec getClosestPoint(const snVec&) const;

		//Return the feature the farthest along the normal.
		//_n : direction along which the search of the farthest feature has to be done.
		//_polygon : array of 4 vectors filled by the function containing the feature. 
		//_count : the number of vectors in the features. Always 4.
		//_featureId : unique id of the feature.
		void getClosestFeature(const snVec& _n, snVec* const _polygon, unsigned int& _count, unsigned int& _featureId) const;

		//return the normal in workd coordinate of the face
		snVec getFeatureNormal(unsigned int _featureId) const;

		//Compute the bounding volume for this collider
		void computeAABB(snAABB * const _boundingVolume) const;

		//////////////////////////////////////////////////////////////////////
		// Interface for SAT algorithm										//
		//////////////////////////////////////////////////////////////////////

		//Fill in an array with all the normals of the obb
		int getUniqueNormals(snVec* _arrayNormals, int _arraySize) const;

		//Make a projection of the obb onto an axis and get the min and max distance along this axis.
		void projectToAxis(const snVec& _axis, float& _min, float& _max) const;

		//////////////////////////////////////////////////////////////////////
		// Interface for GJK algorithm										//
		//////////////////////////////////////////////////////////////////////

		//Return the farthest point in the direction provided as parameter.
		snVec support(const snVec& _direction) const;

		//Return any point making the collider
		snVec anyPoint() const;

	private:
		void computeVertices();

		
	};
}

#endif // SN_OBB_H