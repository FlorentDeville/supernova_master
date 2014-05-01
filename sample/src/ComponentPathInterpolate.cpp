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

#include "ComponentPathInterpolate.h"
#include "snActorDynamic.h"

#include "snTimer.h"
using Supernova::snTimer;

namespace Devil
{
	//Construct an instance of the class ComponentFollowPath
	ComponentPathInterpolate::ComponentPathInterpolate(snActorDynamic* _actor, bool _loop) : m_actor(_actor), m_loop(_loop),
		m_path(), m_nextWaypoint(1), m_previousWaypoint(0), m_startTime(-1)
	{
		m_isActive = true;
	}

	//Clean allocation made by the class ComponentFollowPath
	ComponentPathInterpolate::~ComponentPathInterpolate()
	{
		for (vector<WaypointTime*>::iterator i = m_path.begin(); i != m_path.end(); ++i)
		{
			delete *i;
		}

		m_path.clear();
	}

	//Update the component.
	//It updated the position of the actor
	void ComponentPathInterpolate::update(float _dt)
	{
		//The next waypoint does not exist
		if (m_nextWaypoint >= m_path.size())
			return;

		//if the time is negative, set it to the current time
		if (m_startTime < 0)
			m_startTime = snTimer::convertElapsedTickCountInSeconds(snTimer::getCurrentTick());

		//compute the paramter of interpolation : waypoint time / elapsed time
		double elapsedTime = snTimer::convertElapsedTickCountInSeconds(snTimer::getCurrentTick()) - m_startTime;
		float _param = float(elapsedTime / m_path[m_nextWaypoint]->m_time);

		//if the param is bigger than 1 then we go to the next waypoint
		if (_param > 1)
		{
			_param = _param - 1;

			//go to the next waypoint
			++m_nextWaypoint;
			++m_previousWaypoint;
			m_startTime = snTimer::convertElapsedTickCountInSeconds(snTimer::getCurrentTick());

			//if we can loop then wrap around
			if (m_loop)
			{
				m_nextWaypoint = m_nextWaypoint % m_path.size();
				m_previousWaypoint = m_previousWaypoint % m_path.size();
			}
		}

		//compute the next position
		snVector4f nextPosition = snVector4f::cosInterpolation(m_path[m_previousWaypoint]->m_position,
			m_path[m_nextWaypoint]->m_position, _param);

		//set the position
		m_actor->setLinearVelocity((nextPosition - m_actor->getPosition()) * (1.f / _dt));
	}

	//Do nothing
	void ComponentPathInterpolate::render()
	{}

	//Add a waypoint to the path in the last position
	void ComponentPathInterpolate::addWaypoint(const snVector4f& _position, float _time)
	{
		WaypointTime* newWaypoint = new WaypointTime();
		newWaypoint->m_position = _position;
		newWaypoint->m_time = _time;
		m_path.push_back(newWaypoint);
	}
}