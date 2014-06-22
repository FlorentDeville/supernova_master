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

#ifndef SN_I_CONSTRAINT_H
#define SN_I_CONSTRAINT_H

#include "snVec.inl"
#include "snGlobals.h"

namespace Supernova
{
	//Interface to represent a constraint for one or several bodies.
	class SN_ALIGN snIConstraint
	{
	protected:

		//Indicate if this constraint is currently active.
		bool m_active;

	public:

		//Default constructor
		snIConstraint() : m_active(false){}

		//Default destructor
		virtual ~snIConstraint(){}

		//Prepare the constraint to be resolved precomputing all values not changing during the resolve step.
		//_dt is the time step, it is used to compute baumgarte stabilization
		virtual void prepare(float _dt) = 0;

		//Resolve the constraint by computing a new velocity for the bodies.
		virtual void resolve() = 0;

		//Overridden new operator to create scene with correct alignement.
		void* operator new(size_t _count)
		{
			return _aligned_malloc(_count, SN_ALIGN_SIZE);
		}

		//Overridden delete operator to delete using the correct alignement.
		void operator delete(void* _p)
		{
			_aligned_free(_p);
		}

		void setIsActive(bool _isActive){ m_active = _isActive; }

		bool getIsActive() const{ return m_active; }
	};
}

#endif //SN_I_CONSTRAINT_H