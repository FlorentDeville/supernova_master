#pragma once
#ifndef INPUT_H
#define INPUT_H
#include <Windows.h>
#include <DirectXMath.h>
using DirectX::XMFLOAT2;

namespace Devil
{
	class Input
	{
	private:
		bool m_keys[256];

		//Displacement of the mouse since the last frame.
		XMFLOAT2 m_mouseDelta;

		//Position of the center of the window
		XMFLOAT2 m_wndCenter;

		//Handle to the window
		HWND m_windowHandle;

	public:
		Input();
		virtual ~Input();

		void initialize(HWND _hwnd);

		//Update the state of the inputs.
		void update();

		void keyDown(unsigned int);
		void keyUp(unsigned int);

		bool isKeyDown(unsigned int);

		//Return the mouse displacement since the last frame.
		const XMFLOAT2& getMouseDelta() const;

		//Reset the mouse position to the center of the window.
		void resetMousePosition();
	};
}

#endif