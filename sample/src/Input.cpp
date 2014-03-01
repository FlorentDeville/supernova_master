#include "Input.h"

#include "Graphics.h"

namespace Devil
{

	Input::Input()
	{
	}


	Input::~Input()
	{
	}

	void Input::initialize(HWND _hwnd)
	{
		m_windowHandle = _hwnd;

		// Initialize all the keys to being released and not pressed.
		for ( int i = 0; i<256; i++)
		{
			m_keys[i] = false;
		}

		//clip the mouse to the window so it does not come out of it.
		RECT WindowRect;
		GetWindowRect(m_windowHandle, &WindowRect);
		ClipCursor(&WindowRect);

		//set the initial mouse position
		m_wndCenter.x = GRAPHICS->getScreenWidth() * 0.5f;
		m_wndCenter.y = GRAPHICS->getScreenHeight() * 0.5f;

		resetMousePosition();
		
		return;
	}

	void Input::update()
	{
		//get the mouse position
		POINT pt;
		GetCursorPos(&pt);

		//convert mouse position from screen to window
		ScreenToClient(m_windowHandle, &pt);

		//compute the delta
		m_mouseDelta.x = pt.x - m_wndCenter.x;
		m_mouseDelta.y = pt.y - m_wndCenter.y;

		//reset the mouse position to the center of the window.
		HWND focusedWindow = GetFocus();
		if (focusedWindow == m_windowHandle)
			resetMousePosition();
	}

	void Input::keyDown(unsigned int input)
	{
		// If a key is pressed then save that state in the key array.
		m_keys[input] = true;
		return;
	}


	void Input::keyUp(unsigned int input)
	{
		// If a key is released then clear that state in the key array.
		m_keys[input] = false;
		return;
	}


	bool Input::isKeyDown(unsigned int key)
	{
		// Return what state the key is in (pressed/not pressed).
		return m_keys[key];
	}

	const XMFLOAT2& Input::getMouseDelta() const
	{
		return m_mouseDelta;
	}

	void Input::resetMousePosition()
	{
		POINT pt;
		pt.x = m_wndCenter.x;
		pt.y = m_wndCenter.y;
		ClientToScreen(m_windowHandle, &pt);

		SetCursorPos(pt.x, pt.y);
	}
}