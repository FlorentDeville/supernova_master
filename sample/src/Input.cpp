#include "Input.h"
#include "X360Controller.h"
#include "KeyboardMouse.h"

#include "Graphics.h"

#include <tuple>

namespace Devil
{
	//Initialize static instance
	InputManager* InputManager::m_instance = 0;

	InputManager::InputManager() : m_mouseWheel(0)
	{
	}

	InputManager::~InputManager()
	{

		for (vector<IDevice*>::iterator i = m_devices.begin(); i != m_devices.end(); ++i)
		{
			if ((*i) != 0)
			{
				delete *i;
				*i = 0;
			}
		}
	}

	InputManager* InputManager::getInstance()
	{
		if (m_instance == 0)
			m_instance = new InputManager();

		return m_instance;
	}

	bool InputManager::initialize(HWND _hwnd)
	{
		m_windowHandle = _hwnd;

		// Initialize all the keys to being released and not pressed.
		for ( int i = 0; i<256; i++)
		{
			m_keys[i] = false;
		}

		//set the initial mouse position
		m_wndCenter.x = GRAPHICS->getScreenWidth() * 0.5f;
		m_wndCenter.y = GRAPHICS->getScreenHeight() * 0.5f;

		//resetMousePosition();

		//Create the device for an X360 controller
		X360Controller* new360Device = new X360Controller();
		m_devices.push_back(new360Device);
		
		//Create the device for keyboard and mouse
		m_keyboardMouseDevice = new KeyboardMouse();
		m_devices.push_back(m_keyboardMouseDevice);

		return true;
	}

	void InputManager::shutdown()
	{
		assert(m_instance != 0);
		delete m_instance;
		m_instance = 0;
	}

	void InputManager::update()
	{
		for (vector<IDevice*>::iterator i = m_devices.begin(); i != m_devices.end(); ++i)
		{
			if ((*i) != 0)
			{
				(*i)->update();
			}
		}
	}

	void InputManager::keyDown(unsigned int input)
	{
		// If a key is pressed then save that state in the key array.
		m_keys[input] = true;
		return;
	}


	void InputManager::keyUp(unsigned int input)
	{
		// If a key is released then clear that state in the key array.
		m_keys[input] = false;
		return;
	}

	bool InputManager::isKeyDown(unsigned int key) const
	{
		// Return what state the key is in (pressed/not pressed).
		return m_keys[key];
	}

	float InputManager::getMessage(InputMessage _message) const
	{
		for (vector<IDevice*>::const_iterator i = m_devices.begin(); i != m_devices.end(); ++i)
		{
			if (*i == 0)
			{
				continue;
			}

			if (!(*i)->isConnected())
			{
				continue;
			}

			if (!(*i)->anyMessageAvailable())
			{
				continue;
			}

			return (*i)->getMessage(_message);
		}

		return 0;
	}

	KeyboardMouse* InputManager::getKeyBoardMouseDevice() const
	{
		return m_keyboardMouseDevice;
	}
}