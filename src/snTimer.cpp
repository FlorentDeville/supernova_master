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

#include "snTimer.h"

#include <Windows.h>

namespace Supernova
{
	//Initialize the frequency
	long long snTimer::m_frequency = snTimer::getFrequency();

	//Return the current tick count
	long long snTimer::getCurrentTick()
	{
		LARGE_INTEGER timer;
		QueryPerformanceCounter(&timer);
		return timer.QuadPart;
	}

	uint64_t snTimer::rdtsc()
	{
		return __rdtsc();
	}

	//Compute the elapsed number of tick between the parameter _startTime and the current time.
	long long snTimer::getElapsedTickCount(long long _startTime)
	{
		LARGE_INTEGER timer;
		QueryPerformanceCounter(&timer);
		return timer.QuadPart - _startTime;
	}
	//Convert a tick count into seconds using the frequency
	float snTimer::convertElapsedTickCountInSeconds(long long _elapsedTime)
	{
		return (float)_elapsedTime / (float)m_frequency;
	}

	//Convert a tick count into milliseconds using the frequency
	float snTimer::convertElapsedTickCountInMilliSeconds(long long _elapsedTime)
	{
		return (float)_elapsedTime / ((float)m_frequency * 0.001f);
	}

	//Convert a tick count into micro seconds using the frequency
	float snTimer::convertElapsedTickCountInMicroSeconds(long long _elapsedTime)
	{
		return (float)_elapsedTime / ((float)m_frequency * 0.001f * 0.001f);
	}


	//Return the number of tick per seconds
	long long snTimer::getFrequency()
	{
		LARGE_INTEGER frequency;
		QueryPerformanceFrequency(&frequency);
		return frequency.QuadPart;
	}
}