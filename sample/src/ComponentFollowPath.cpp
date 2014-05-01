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

#include "ComponentFollowPath.h"
#include "snActorDynamic.h"

namespace Devil
{
	//Construct an instance of the class ComponentFollowPath
	ComponentFollowPath::ComponentFollowPath(snActorDynamic* _actor, bool _loop) : m_actor(_actor), m_loop(_loop), 
		m_path(), m_nextWaypoint(1), m_previousWaypoint(0)
	{
		m_isActive = true;
	}

	//Clean allocation made by the class ComponentFollowPath
	ComponentFollowPath::~ComponentFollowPath()
	{
		for (vector<Waypoint*>::iterator i = m_path.begin(); i != m_path.end(); ++i)
		{
			delete *i;
		}

		m_path.clear();
	}

	//Update the component.
	//It updated the position of the actor
	void ComponentFollowPath::update(float _dt)
	{
		//The next waypoint does not exist
		if (m_nextWaypoint >= m_path.size())
			return;

		//compute the direction
		snVector4f direction = m_path[m_nextWaypoint]->m_position - m_path[m_previousWaypoint]->m_position;
		float length = direction.squareNorme();
		direction.normalize();

		//compute the next position
		snVector4f nextPosition = m_actor->getPosition() + direction * m_path[m_nextWaypoint]->m_speed * _dt;

		//check if we went too far
		//compare the distance between the two waypoints and the distance between the first waypoint and the computed position.
		float actorDistance = (nextPosition - m_path[m_previousWaypoint]->m_position).squareNorme();
		if (length < actorDistance)
		{
			//we went too far then set the position to the waypoint to reach.
			nextPosition = m_path[m_nextWaypoint]->m_position;

			//go to the next waypoint
			++m_nextWaypoint;
			++m_previousWaypoint;

			//if we can loop then wrap around
			if (m_loop)
			{
				m_nextWaypoint = m_nextWaypoint % m_path.size();
				m_previousWaypoint = m_previousWaypoint % m_path.size();
			}
		}

		//set the position
		m_actor->setLinearVelocity((nextPosition - m_actor->getPosition()) * (1.f/_dt));	
	}

	//Do nothing
	void ComponentFollowPath::render()
	{}

	//Add a waypoint to the path in the last position
	void ComponentFollowPath::addWaypoint(const snVector4f& _position, float _speed)
	{
		Waypoint* newWaypoint = new Waypoint();
		newWaypoint->m_position = _position;
		newWaypoint->m_speed = _speed;
		m_path.push_back(newWaypoint);
	}
}