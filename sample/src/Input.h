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
		//singleton instance
		static Input* m_instance;

		//Array containing state of each key of keyboard
		bool m_keys[256];

		//Displacement of the mouse since the last frame.
		XMFLOAT2 m_mouseDelta;

		//displacement of the mouse wheel positive being forward and negative being backward.
		float m_mouseWheel;

		//Position of the center of the window
		XMFLOAT2 m_wndCenter;

		//Handle to the window
		HWND m_windowHandle;

	private:
		Input();
		virtual ~Input();

	public:
		static Input* getInstance();
		bool initialize(HWND _hwnd);
		void shutdown();

		//Update the state of the inputs.
		void update();

		void keyDown(unsigned int);
		void keyUp(unsigned int);

		//Set the mouse wheel displacement
		void setMouseWheel(float _displacement);

		bool isKeyDown(unsigned int);

		//Return the mouse displacement since the last frame.
		const XMFLOAT2& getMouseDelta() const;

		//Get the mouse wheel displacement
		float getMouseWheel() const;

		//Reset the mouse position to the center of the window.
		void resetMousePosition();
	};

	//get the Input singleton
#define INPUT Input::getInstance()

}

#endif