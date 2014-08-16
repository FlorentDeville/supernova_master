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

#ifndef COMPONENT_BACKGROUND_H
#define COMPONENT_BACKGROUND_H

#include "IComponent.h"
#include "snVec.h"
#include "snMatrix44f.h"

namespace Supernova
{
	class snRigidbody;
}
using namespace Supernova;

namespace Devil
{
	class ComponentBackground : public IComponent
	{
	private:
		//Actor representing the background
		snRigidbody* m_background;

		//Actor representing the point which all rotation axis go through
		snRigidbody* m_origin;

		//Background transform
		snMatrix44f m_transform;

		snVec m_forward;

		snVec m_left;
	public:

		ComponentBackground(snRigidbody* _background, snRigidbody* _origin, const snVec& _initialTranslation, const snVec& _initialOrientation);

		~ComponentBackground();

		void update(float _dt);

		void render();

		void* operator new(size_t _count);

		void operator delete(void* _p);

	private:

		void computeOriginFrame(const snRigidbody* _origin, snVec& _forward, snVec& _left);
	};
}
#endif //ifndef COMPONENT_BACKGROUND_H