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

#ifndef SN_FEATURE_CLIPPING_H
#define SN_FEATURE_CLIPPING_H

#include "snTypes.h"

namespace Supernova
{
	class snICollider;

	//Find the collision points of two intersecting colliders using the clipping plan algorithms.
	class snFeatureClipping
	{
	public:

		//default constructor
		snFeatureClipping();

		//Default destructor
		virtual ~snFeatureClipping();

		//Find the collision patch of two intersecting colliders.
		bool findContactPatch(const snICollider& _c1, const snICollider& _c2, const snVec& _normal, 
			snVecVector& _patch, vector<float>& _patchPenetrations) const;

		//Find the collision patch of two features.
		// _feature1 : array of vertex for the first feature.
		// _featureSize1 : number of vertex in first feature.
		// _featureNormal1 : normal of the first feature pointing outside.
		// _feature2 : array of vertex for the second feature.
		// _featureSize2 : number of vertex in second feature.
		// _featureNormal2 : normal of the second feature pointing outside.
		// _normal : collision normal going from the second collider to the first one.
		// _patch : the result of the clipping operation.
		// _patchPenetrations : penetration depth for each vertex of the patch.
		bool findContactPatch(snVec* const _feature1, unsigned int _featureSize1, const snVec& _featureNormal1,
			snVec* const _feature2, unsigned int _featureSize2, const snVec& _featureNormal2, 
			const snVec& _normal, snVecVector& _patch, vector<float>& _patchPenetrations) const;

	private:

		//Clip a polygon to keep only the points on the back side of a plan.
		void clipPolygon(const snVecVector& _polygon, const snVec& _n, float _d, snVecVector& _clippedPolygon) const;

		//Compute the intersection between a plane and a line.
		void computeIntersection(const snVec& _n, float _d, const snVec& _start, const snVec& _end, snVec& _intersection) const;

		void test_newClipping(snVec* _reference, const snVec& referenceNormal, snVec* _incident, int _incidentSize, snVecVector& _result) const;

		void clipVertexVertex(snVec* _feature1, snVec* _feature2, const snVec& _n, snVecVector& _patch, 
			vector<float>& _patchPenetrations) const;

		void clipVertexEdge(snVec* _featureVertex, snVec* _featureEdge, const snVec& _n, snVecVector& _patch, 
			vector<float>& _patchPenetrations) const;

		void clipVertexFace(snVec* _featureVertex, snVec* _featureFace, const snVec& _n, snVecVector& _patch, 
			vector<float>& _patchPenetrations) const;

		void clipEdgeEdge(snVec* _featureEdge1, snVec* _featureEdge2, const snVec& _n, snVecVector& _patch, 
			vector<float>& _patchPenetrations) const;

		bool clipFaceFace(snVec* _reference, const snVec& _referenceFeatureNormal, snVec* _incident, int _incidentSize, snVecVector& _patch,
			vector<float>& _patchPenetrations) const;
	};
}

#endif //SN_FEATURE_CLIPPING_H