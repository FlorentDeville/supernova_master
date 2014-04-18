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
#ifndef COMPONENT_FLOATING_TEXT_H
#define COMPONENT_FLOATING_TEXT_H

#include "IComponent.h"

#include <vector>
using std::vector;

#include <tuple>
using std::tuple;

#include <DirectXMath.h>
using DirectX::XMFLOAT2;

#include <string>
using std::wstring;

#include "Graphics.h"

namespace Devil
{
	class IWorldEntity;

	//////////////////////////////////////////////////////////
	// Display text on screen, on an entity.
	// T is the type of the object containing the value to display.
	// R is the type of the value to display.
	//////////////////////////////////////////////////////////
	template<class T, class R>
	class ComponentFloatingText : public IComponent
	{
	private:

		//The entity the text must be above
		const IWorldEntity* m_anchor;

		//Defines the signature of the getter to use to retrieve the value to display
		typedef R(T::*getValue)() const;

		//Defines a tuple containing the necessary information to retrieve the value to display.
		typedef tuple<wstring, T*, getValue> ItemTuple;

		//List of items to display.
		//It containd tuples of 3 elements. The first is the name to display. The second one is the object containing the value to display.
		//The third one is a pointer to the getter to retrieve the value.
		vector<ItemTuple> m_items;

		//The position of the text in screen coordinate
		XMFLOAT2 m_textPosition;

		//Offset position for the displayed text.
		XMFLOAT2 m_offset;

	public:
		//Constructor
		ComponentFloatingText();

		//Destructor
		virtual ~ComponentFloatingText();

		//Update the component
		void update();

		//Render the text
		void render();

		//Set the entity used as an anchor. The text will be stuck to this entity
		void setAnchor(const IWorldEntity* _anchor);

		//Set the offset to add to the position of text
		void setOffset(const XMFLOAT2& _offset);

		// Add an item to display
		// _name is the title to display before the value.
		// _obj is a pointer to the object containing the value to display
		// _getter is a pointer to a const getter function.
		void addItem(const wstring& _name, T* _obj, getValue _getter);
	};

	template<class T, class R> ComponentFloatingText<T, R>::ComponentFloatingText() : m_offset(0, 0)
	{

	}

	template<class T, class R> ComponentFloatingText<T, R>::~ComponentFloatingText()
	{

	}

	template<class T, class R> void ComponentFloatingText<T, R>::update()
	{
		//convert the world position of the anchor entity into screen position
		XMVECTOR screen = GRAPHICS->worldToScreen(m_anchor->getPosition());
		m_textPosition.x = screen.m128_f32[0];
		m_textPosition.y = screen.m128_f32[1];
	}

	template<class T, class R> void ComponentFloatingText<T, R>::render()
	{
		//start to write sprites
		GRAPHICS->spriteBeginRender();

		const int LINEHEIGHT = 17;
		const float SCALE = 0.5f;

		float offsetHeight = 0;
		XMFLOAT2 pos;
		pos.x = m_textPosition.x + m_offset.x;
		pos.y = m_textPosition.y + m_offset.y;

		//loop through each item
		for (vector<ItemTuple>::iterator i = m_items.begin(); i != m_items.end(); ++i)
		{
			//retrive information from the tuple
			wstring title = std::get<0>(*i);
			T* object = std::get<1>(*i);
			getValue functionPtr = std::get<2>(*i);

			//get the value to display
			R value = (object->*functionPtr)();

			//write text
			pos.y += offsetHeight;
			GRAPHICS->writeText(title + L":" + std::to_wstring(value), pos, SCALE);
			offsetHeight += LINEHEIGHT;
		}

		//turn of sprites writing
		GRAPHICS->spriteEndRender();
	}

	template<class T, class R> void ComponentFloatingText<T, R>::setAnchor(const IWorldEntity* _anchor)
	{
		m_anchor = _anchor;
	}

	template<class T, class R> void ComponentFloatingText<T, R>::setOffset(const XMFLOAT2& _offset)
	{
		m_offset = _offset;
	}

	template<class T, class R>void ComponentFloatingText<T, R>::addItem(const wstring& _name, T* _obj, getValue _getter)
	{
		m_items.push_back(std::make_tuple(_name, _obj, _getter));
	}
}
#endif //ifndef COMPONENT_FLOATING_TEXT_H