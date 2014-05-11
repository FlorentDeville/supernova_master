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

#ifndef PATH_EXPLORER_H
#define PATH_EXPLORER_H

#include <vector>
using std::vector;

#include "snVec.inl"
using Supernova::snVec;

#include "snGlobals.h"

namespace Supernova
{
	class snMatrix44f;
}
using Supernova::snMatrix44f;

namespace Devil
{
	//Definition of the callback function called for every point on the path.
	typedef void(*OnPathCallback)(const snMatrix44f&);

	class PathExplorerWaypoint
	{
	public:
		snVec m_controlPoint;
		float m_distance;

	public:
		void* operator new(size_t _count)
		{
			return _aligned_malloc(_count, SN_ALIGN_SIZE);
		}

		void operator delete(void* _p)
		{
			_aligned_free(_p);
		}
	};

	//Helper class used to create a path with catmull rom interpolation. A set of points are created along the path, separated by a given
	// distance. A callback is called for each point found.
	class PathExplorer
	{
	private:

		//Control points defining the path.
		vector<PathExplorerWaypoint*> m_path;

		//function called every time a point is found along the path.
		OnPathCallback m_callback;

		//Indicates if the path loops.
		bool m_loop;

		//Indices of waypoint used to interpolate the path
		unsigned int m_indices[4];

	public:
		PathExplorer(bool _loop);
		~PathExplorer();

		void addWaypoint(const snVec& _controlPoint, float _distance);
		void setCallback(OnPathCallback _callback);

		void run();

	private:
		bool nextPoint(const snVec& _previousPoint, float& _t, snMatrix44f& _frenet);

	};
}

#endif //ifndef PATH_EXPLORER_H