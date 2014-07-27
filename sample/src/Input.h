#pragma once
#ifndef INPUT_H
#define INPUT_H
#include <Windows.h>
#include <DirectXMath.h>
using DirectX::XMFLOAT2;

#include "IDevice.h"
using namespace Devil::Input;
using namespace Devil::Input::Device;

namespace Devil
{
	namespace Input
	{
		namespace Device
		{
			class KeyboardMouse;
		}
	}
	using Input::Device::KeyboardMouse;

	class InputManager
	{
	private:
		//singleton instance
		static InputManager* m_instance;

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

		//List of devices.
		vector<IDevice*> m_devices;

		//Pointer to the device for the keyboard and mouse.
		KeyboardMouse* m_keyboardMouseDevice;

	private:
		InputManager();
		virtual ~InputManager();

	public:
		static InputManager* getInstance();
		bool initialize(HWND _hwnd);
		void shutdown();

		//Update the state of the inputs.
		void update();

		void keyDown(unsigned int);
		void keyUp(unsigned int);

		bool isKeyDown(unsigned int) const;

		//Return whether or not an InputMessage was received.
		// _message : what message to check.
		// return : 0 if no message. A float between -1 and 1 otherwise.
		float getMessage(InputMessage _message) const;

		//Return a pointer to the keyboard mouse device.
		KeyboardMouse* getKeyBoardMouseDevice() const;
	};

	//get the Input singleton
#define INPUT InputManager::getInstance()

}

#endif