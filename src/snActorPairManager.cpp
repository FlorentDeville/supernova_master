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

#include "snActorPairManager.h"
#include "snActorPair.h"

namespace Supernova
{
	//Constructor
	snActorPairManager::snActorPairManager() : m_pairs(), m_currentPairId(0)
	{}

	//Destructor
	snActorPairManager::~snActorPairManager()
	{
		for (vector<snActorPair*>::iterator i = m_pairs.begin(); i != m_pairs.end(); ++i)
		{
			delete *i;
		}
		m_pairs.clear();
	}

	//Prepare the manager for the broad phase
	void snActorPairManager::preBroadPhase()
	{
		m_currentPairId = 0;
	}

	//Clean up after the broad phase
	void snActorPairManager::postBroadPhase()
	{

	}

	//Return a pointer to an available snActorPair.
	snActorPair* snActorPairManager::getAvailablePair()
	{
		if (m_currentPairId >= m_pairs.size())
		{
			snActorPair* currentPair = new snActorPair();
			m_pairs.push_back(currentPair);
		}

		return m_pairs[m_currentPairId++];
	}

	//Return the vector of pairs
	const vector<snActorPair*>& snActorPairManager::getPairs() const
	{
		return m_pairs;
	}

	//Return the number of active pairs.
	unsigned int snActorPairManager::getActivePairsCount() const
	{
		return m_currentPairId;
	}
}