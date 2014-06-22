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
#include "snFeatureClipping.h"
#include "snICollider.h"

#include <assert.h>

using namespace Supernova::Vector;

namespace Supernova
{
	snFeatureClipping::snFeatureClipping(){}

	snFeatureClipping::~snFeatureClipping(){}

	bool snFeatureClipping::findContactPatch(const snICollider& _c1, const snICollider& _c2, const snVec& _normal,
		snVecVector& _patch, vector<float>& _patchPenetrations) const
	{
		//find the closest polygon
		snVec closestPolygonB1[4];
		snVec closestPolygonB2[4];

		int FaceIdB1, FaceIdB2;
		_c1.getClosestPolygonProjected(_normal, closestPolygonB1, FaceIdB1);
		_c2.getClosestPolygonProjected(_normal * -1, closestPolygonB2, FaceIdB2);

		//find the reference polygon. It is the polygon the most orthogonal to the normal. So the dot product between the poly normal and the
		//collision normal is the bigger.
		snVec edge1B1 = closestPolygonB1[1] - closestPolygonB1[0];
		snVec edge2B1 = closestPolygonB1[1] - closestPolygonB1[2];
		snVec NormalB1 = snVec3Cross(edge1B1, edge2B1);
		snVec3Normalize(NormalB1);
		snVec dotB1 = snVec3Dot(NormalB1, _normal);

		snVec edge1B2 = closestPolygonB2[1] - closestPolygonB2[0];
		snVec edge2B2 = closestPolygonB2[1] - closestPolygonB2[2];
		snVec NormalB2 = snVec3Cross(edge1B2, edge2B2);
		snVec3Normalize(NormalB2);
		snVec dotB2 = snVec3Dot(NormalB2, _normal);

		snVec* Reference = 0;
		snVec* Incident = 0;
		const snICollider* ReferenceBox;
		const snICollider* IncidentBox;
		int ReferenceFaceId, IncidentFaceId;
		if (snVec3Inferior(dotB1, dotB2))
		{
			Reference = closestPolygonB2;
			Incident = closestPolygonB1;
			ReferenceBox = &_c2;
			IncidentBox = &_c1;
			ReferenceFaceId = FaceIdB2;
			IncidentFaceId = FaceIdB1;
		}
		else
		{
			Reference = closestPolygonB1;
			Incident = closestPolygonB2;
			ReferenceBox = &_c1;
			IncidentBox = &_c2;
			ReferenceFaceId = FaceIdB1;
			IncidentFaceId = FaceIdB2;
		}

		//get the list of plan to use for clipping
		const int* adjacentFaces = ReferenceBox->getAdjacentFaces(ReferenceFaceId);
		snVecVector incidentPolygon;
		incidentPolygon.reserve(4);
		for (int i = 0; i < 4; ++i)
			incidentPolygon.push_back(Incident[i]);

		//loop through each adjacent faces
		for (int i = 0; i < 4; ++i)
		{
			snVec adjacentNormal = ReferenceBox->getWorldNormalOfFace(adjacentFaces[i]);
			snVec vertexInPlan = ReferenceBox->getWorldVertexOfFace(adjacentFaces[i]);

			float d = snVec4GetX(snVec3Dot(adjacentNormal, vertexInPlan));

			//clip the incident polygon using the plane
			snVecVector clipped;
			clipPolygon(incidentPolygon, adjacentNormal, d, clipped);
			incidentPolygon.swap(clipped);
		}

		if (incidentPolygon.size() == 0)
		{
			return false;
		}

		assert(incidentPolygon.size() > 0);

		
		//if more than one vertices left, clip using the reference plane
		if (incidentPolygon.size() > 0)
		{
			
			snVec adjacentNormal = ReferenceBox->getWorldNormalOfFace(ReferenceFaceId);
			snVec vertexInPlan = ReferenceBox->getWorldVertexOfFace(ReferenceFaceId);
			snVec d = snVec3Dot(adjacentNormal, vertexInPlan);

			//Reserve space to avoid several dynamic allocations
			_patchPenetrations.reserve(incidentPolygon.size());
			_patch.reserve(incidentPolygon.size());

			for (snVecVectorConstIterator vertex = incidentPolygon.cbegin(); vertex != incidentPolygon.cend(); ++vertex)
			{
				snVec dot = d - snVec3Dot(*vertex, adjacentNormal);
				//only keep vertices on the good side.
				if (snVec3SuperiorOrEqual(dot, snVec4Set(0.0f)))
				{
					_patch.push_back(*vertex);
					_patchPenetrations.push_back(snVec4GetX(dot));
				}
			}
		}

		if (_patch.size() == 0)
		{
			return false;
		}

		assert(_patch.size() > 0);

		return true;
	}

	void snFeatureClipping::clipPolygon(const snVecVector& _polygon, const snVec& _n, float _d, snVecVector& _clippedPolygon) const
	{
		if (_polygon.size() == 0)
			return;

		//Reserve enough space to contain all the points of the source polygon to avoid several dynamic allocation later.
		_clippedPolygon.reserve(_polygon.size());

		snVec firstPoint = _polygon[_polygon.size() - 1];
		for (snVecVectorConstIterator i = _polygon.cbegin(); i != _polygon.cend(); ++i)
		{
			//check if the vertices are on the good side of the plan
			float d1 = snVec4GetX(snVec3Dot(firstPoint, _n)) - _d;
			float d2 = snVec4GetX(snVec3Dot(*i, _n)) - _d;

			const float planeThickness = 0.0f;
			bool isP1OnCorrectSide = d1 <= planeThickness;
			bool isP2OnCorrectSide = d2 <= planeThickness;

			if (isP1OnCorrectSide && isP2OnCorrectSide) //correct side
			{
				_clippedPolygon.push_back(firstPoint);
			}
			else if (isP1OnCorrectSide) //only the first point on the correct side
			{
				//we save the first point and compute the intersection
				snVec newVertex;
				computeIntersection(_n, _d, firstPoint, *i, newVertex);
				_clippedPolygon.push_back(firstPoint);
				_clippedPolygon.push_back(newVertex);
			}
			else if (isP2OnCorrectSide) //only the second point on the correct side
			{
				//compute the intersection. do not save the first point as it must have been saved during a previous iteration.
				snVec newVertex;
				computeIntersection(_n, _d, firstPoint, *i, newVertex);
				_clippedPolygon.push_back(newVertex);
			}

			firstPoint = *i;
		}
	}

	void snFeatureClipping::computeIntersection(const snVec& _n, float _d, const snVec& _start, const snVec& _end, 
		snVec& _intersection) const
	{
		//The intersection point is on the line so it is solution of I = start + (end - start) * k (1).
		//It is also on the plane so it solution of N.I = d (2).
		//Put (1) in (2) then solve for k and you end up with k= (d - N.Start) / N.(end - start)
		snVec edge = _end - _start;
		snVec3Normalize(edge);

		snVec NDotS = snVec3Dot(_n, _start);
		snVec NDotEdge = snVec3Dot(_n, edge);
		//assert(NDotEdge != 0.f);

		snVec parameter = (snVec3Set(_d) - NDotS) / NDotEdge;

		_intersection = _start + edge * parameter;
	}
}