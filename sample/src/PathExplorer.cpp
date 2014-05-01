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

#include "PathExplorer.h"
#include "snMath.h"
#include "snMatrix44f.h"
using namespace Supernova;

namespace Devil
{
	PathExplorer::PathExplorer(bool _loop) : m_loop(_loop), m_path(), m_callback(0){}

	PathExplorer::~PathExplorer()
	{
		for (vector<PathExplorerWaypoint*>::iterator i = m_path.begin(); i != m_path.end(); ++i)
		{
			delete (*i);
		}
		m_path.clear();
	}

	void PathExplorer::addWaypoint(const snVector4f& _controlPoint, float _distance)
	{
		PathExplorerWaypoint* newWaypoint = new PathExplorerWaypoint();
		newWaypoint->m_controlPoint = _controlPoint;
		newWaypoint->m_distance = _distance;
		m_path.push_back(newWaypoint);
	}

	void PathExplorer::setCallback(OnPathCallback _callback)
	{
		m_callback = _callback;
	}

	void PathExplorer::run()
	{
		//setup indices
		if (m_loop)
			m_indices[0] = m_path.size() - 1;
		else
			m_indices[0] = 0;

		m_indices[1] = 0;
		m_indices[2] = 1;
		m_indices[3] = 2;

		snVector4f previousPoint;

		//loop through waypoint
		for (unsigned int waypointId = 0; waypointId < m_path.size(); ++waypointId)
		{
			snMatrix44f frenet;
			if (waypointId == 0)
			{
				computeFrenetFromCatmullRom(m_path[m_indices[0]]->m_controlPoint,
					m_path[m_indices[1]]->m_controlPoint,
					m_path[m_indices[2]]->m_controlPoint,
					m_path[m_indices[3]]->m_controlPoint,
					0,
					frenet);

				if (m_callback != 0)
					m_callback(frenet);

				previousPoint = frenet[3];
			}

			float t = 0;

			
			while (t < 1)
			{
				bool nextWaypoint = nextPoint(previousPoint, t, frenet);

				//we found the point
				if (!nextWaypoint)
				{
					if (m_callback != 0)
						m_callback(frenet);

					previousPoint = frenet[3];
				}
				else //we need to go to the next waypoint
				{
					t = 1;
				}
			}

			//increase indices to move to the next waypoint
			m_indices[0] = m_indices[1];
			m_indices[1] = m_indices[2];
			m_indices[2] = m_indices[3];
			if (m_loop)
				m_indices[3] = (m_indices[3] + 1) % m_path.size();
			else
			{
				if (m_indices[3] < m_path.size() - 1)
					++m_indices[3];
			}
		}
	}

	bool PathExplorer::nextPoint(const snVector4f& _previousPoint, float& _t, snMatrix44f& _frenet)
	{
		//check if the next point is in the next waypoint
		computeFrenetFromCatmullRom(m_path[m_indices[0]]->m_controlPoint,
			m_path[m_indices[1]]->m_controlPoint,
			m_path[m_indices[2]]->m_controlPoint,
			m_path[m_indices[3]]->m_controlPoint,
			1,
			_frenet);

		float distance = (_frenet[3] - _previousPoint).norme() - m_path[m_indices[2]]->m_distance;
		if (distance < 0)//two close, we need to go to the next waypoint
			return true;


		const float EPSILON = 0.01f;

		distance = 1;

		float currentT = _t;
		bool found = false;
		float minT = _t;
		float maxT = 1;
		while (!found)
		{
			currentT = (minT + maxT) * 0.5f;

			computeFrenetFromCatmullRom(m_path[m_indices[0]]->m_controlPoint,
				m_path[m_indices[1]]->m_controlPoint,
				m_path[m_indices[2]]->m_controlPoint,
				m_path[m_indices[3]]->m_controlPoint,
				currentT,
				_frenet);

			distance = (_frenet[3] - _previousPoint).norme() - m_path[m_indices[2]]->m_distance;
			if (fabsf(distance) <= EPSILON)//the point is close enough
			{
				found = true;
				_t = currentT;
			}
			else
			{
				if (distance < 0)
				{
					minT = currentT;
				}
				else
				{
					maxT = currentT;
				}
			}
		}

		return false;
	}
}