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

#ifndef COMPONENT_FOLLOW_PATH_H
#define COMPONENT_FOLLOW_PATH_H

#include "IComponent.h"

#include "snGlobals.h"

#include "snVec.h"
using Supernova::snVec;

#include <vector>
using std::vector;

namespace Supernova
{
	class snActorDynamic;
}

using Supernova::snActorDynamic;


namespace Devil
{
	class Waypoint
	{
	public:
		//Position of the waypoint
		snVec m_position;

		//Speed of the actor to reach this waypoint
		float m_speed;

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

	//////////////////////////////////////////////////////////
	// Defines a path using waypoints the entity will follow.
	//////////////////////////////////////////////////////////
	class ComponentFollowPath : public IComponent
	{
	private:
		//The actor to move
		snActorDynamic* m_actor;

		//The list of waypoint making the path to follow
		vector<Waypoint*> m_path;

		//Flag to indicate if the actor as to loop the path or stop when it reaches the end.
		bool m_loop;

		//Id in the vector m_path of the next waypoint to reach
		unsigned int m_nextWaypoint;

		//Id in the vector m_path of the previous waypoint.
		unsigned int m_previousWaypoint;

	public:
		//Construct an instance of the class ComponentFollowPath
		ComponentFollowPath(snActorDynamic* _actor, bool _loop);

		//Clean allocation made by the class ComponentFollowPath
		virtual ~ComponentFollowPath();

		//Update the component.
		//It updated the position of the actor
		void update(float _dt);

		//Do nothing
		void render();

		//Add a waypoint to the path in the last position
		void addWaypoint(const snVec& _position, float _speed);
	};

	
}
#endif //ifndef COMPONENT_FOLLOW_PATH_H