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

#ifndef SN_HANDLE_H
#define SN_HANDLE_H

#include "snWorld.h"

namespace Supernova
{
	class snObject;
	class snScene;
	class snActorDynamic;
	class snActorStatic;

	template<class C> class snHandle
	{
	protected:

		//The id of the object in the look up table of Supernova.
		snObjectId m_id;

	public:
		snHandle()
		{
			static_assert(std::is_base_of<snObject, C>::value, "The template parameter of class snHandle must be a derived class of snObject.");
			m_id = 0;
		}

		snHandle(snObjectId _id)
		{
			static_assert(std::is_base_of<snObject, C>::value, "The template parameter of class snHandle must be a derived class of snObject.");
			m_id = _id;
		}

		virtual ~snHandle(){}

		C* const getPtr() const { return static_cast<C* const>(snWorld::getInstance()->getObject(m_id)); }

		snObjectId getId() const { return m_id; }

		bool isValid() const { return getPtr() == 0 ? false : true; }

		C* const operator->() const { return getPtr(); }
	};

	typedef snHandle<snScene> snhScene;					//Handle for a scene.
	typedef snHandle<snActorStatic> snhActorStatic;		//Handle for a static actor.
	typedef snHandle<snActorDynamic> snhActorDynamic;	//Handle for a dynamic actor.
}

#endif //ifndef SN_HANDLE_H