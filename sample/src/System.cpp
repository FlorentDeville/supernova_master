#include "System.h"

#include "Input.h"
#include "Graphics.h"
#include "Camera.h"
#include "World.h"
#include "SceneManager.h"



#include "EffectProvider.h"

#include "EntitySphere.h"
#include "EntityBox.h"
#include "EntityPlan.h"
#include "EntityCamera.h"
#include "EntityFixedConstraint.h"

#include "snSphere.h"
#include "snColliderPlan.h"

#include "snScene.h"
#include "snActorDynamic.h"
#include "snFactory.h"
#include "snQuaternion.h"
#include "snFixedConstraint.h"
#include "snDebugger.h"
#include "snTimer.h"

#include <string>
#include <Windowsx.h>

using namespace Supernova;

namespace Devil
{

	System::System() : m_fullScreen(false)
	{
	}

	System::~System()
	{
	}

	bool System::initialize(bool _fullScreen)
	{
		m_fullScreen = _fullScreen;
		m_deltaTime = 1000 / TICK;

		int screenWidth, screenHeight;
		bool result;

		// Initialize the width and height of the screen to zero before sending the variables into the function.
		screenWidth = 0;
		screenHeight = 0;

		// Initialize the windows api.
		initializeWindows(screenWidth, screenHeight);
		SetWindowText(m_hwnd, L"SUPERNOVA Demo1 (Florent Devillechabrol)");
		// Initialize the graphics object.
		result = GRAPHICS->initialize(screenWidth, screenHeight, m_hwnd, _fullScreen, false);
		if (!result)
		{
			return false;
		}
		result = EFFECT->initialize();
		if (!result)
			return false;

		// Initialize the inputs.
		if (!INPUT->initialize(m_hwnd))
			return false;

		//initialize the scene manager
		if (!SCENEMGR->initialize())
			return false;

		//initialize the physics
		result = SUPERNOVA->initialize();
		if (!result)
		{
			return false;
		}

		//Start with the first scene
		//SCENEMGR->createBasicTest();
		//SCENEMGR->createStacking();
		//SCENEMGR->createSceneActorsType();
		//SCENEMGR->createSceneDomino();
		//SCENEMGR->createConstraints();
		//SCENEMGR->createSceneComposite();
		SCENEMGR->createSceneMonkeyBall();

		return true;
	}

	void System::shutdown()
	{
		SUPERNOVA->clean();
		WORLD->shutdown();
		EFFECT->kill();
		GRAPHICS->shutdown();
		INPUT->shutdown();
		SCENEMGR->shutdown();
		
		// Shutdown the window.
		shutdownWindows();

		return;
	}

	void System::run()
	{
		MSG msg;
		bool done;

		// Initialize the message structure.
		ZeroMemory(&msg, sizeof(MSG));

		m_lastTick = 0;

		int fpsCounter = 0;
		int fpsCounterPhysics = 0;
		long long fpsTimeCounter = 0;

		LARGE_INTEGER frequency;
		QueryPerformanceFrequency(&frequency);

		unsigned long tickPerMilliseconds = (unsigned long)(frequency.QuadPart * 0.001f);

		// Loop until there is a quit message from the window or the user.
		done = false;
		while (!done)
		{
			// Handle the windows messages.
			if (PeekMessage(&msg, NULL, 0, 0, PM_REMOVE))
			{
				TranslateMessage(&msg);
				DispatchMessage(&msg);
			}

			// If windows signals to end the application then exit out.
			if (msg.message == WM_QUIT)
			{
				done = true;
			}
			else
			{
				//check if enough time has elapsed
				LARGE_INTEGER currentClock;
				QueryPerformanceCounter(&currentClock);
				unsigned long currentTime = (unsigned long)(currentClock.QuadPart - m_lastTick) / tickPerMilliseconds;
				if (currentTime >= m_deltaTime)
				{
					DEBUGGER->clearWatch();

					float dt = m_deltaTime * 0.001f;
					m_lastTick = currentClock.QuadPart;
					LARGE_INTEGER clockStart;
					QueryPerformanceCounter(&clockStart);

					//Update the inputs
					INPUT->update();

					//update the world
					WORLD->update(dt);

#ifdef SN_DEBUGGER
					long long startTimer = snTimer::getCurrentTick();
#endif //ifdef SN_DEBUGGER
					//update all the scenes (there is just one anyway)
					SUPERNOVA->updateAllScenes(dt);

#ifdef SN_DEBUGGER
					float durationMS = snTimer::convertElapsedTickCountInMilliSeconds(snTimer::getElapsedTickCount(startTimer));
					DEBUGGER->setWatchExpression(L"Physics step (ms)", std::to_wstring(durationMS));
#endif //ifdef SN_DEBUGGER

					fpsCounterPhysics++;

					//Update the scene manager to check if a new scene as to be loaded.
					SCENEMGR->update();

					// Check if the user pressed escape and wants to exit the application.
					if (INPUT->isKeyDown(VK_ESCAPE))
					{
						done = true;
					}

					if (INPUT->isKeyDown('C'))
					{
						WORLD->activateCollisionPoint();
					}
					if (INPUT->isKeyDown('V'))
					{
						WORLD->deactivateCollisionPoint();
					}
				}

				// Do the frame processing for the graphics object.
				WORLD->render();
				++fpsCounter;

				//update fps
				QueryPerformanceCounter(&currentClock);
				currentTime = (unsigned long)(currentClock.QuadPart - fpsTimeCounter) / tickPerMilliseconds;
				if (currentTime >= 1000)
				{
					WORLD->setGraphicsFPS(fpsCounter);
					WORLD->setPhysicsFPS(fpsCounterPhysics);

					fpsTimeCounter = currentClock.QuadPart;
					fpsCounter = 0;	
					fpsCounterPhysics = 0;
				}
			}

		}

		return;

	}

	LRESULT CALLBACK System::messageHandler(HWND hwnd, UINT umsg, WPARAM wparam, LPARAM lparam)
	{
		switch (umsg)
		{
			case WM_MOUSEWHEEL:
			{
				int ds = (int)wparam;
				INPUT->setMouseWheel((float)ds);
				return 0;
			}

			// Check if a key has been pressed on the keyboard.
			case WM_KEYDOWN:
			{
				// If a key is pressed send it to the input object so it can record that state.
				INPUT->keyDown((unsigned int)wparam);
				return 0;
			}

			//Check if a key has been released on the keyboard.
			case WM_KEYUP:
			{
				unsigned int key = (unsigned int)wparam;
				// If a key is released then send it to the input object so it can unset the state for that key.
				INPUT->keyUp(key);
				
				if (key == '1')
				{
					SUPERNOVA->getScene(0)->setCollisionMode(snCollisionMode::snECollisionModeBruteForce);
					SCENEMGR->setCollisionMode(snCollisionMode::snECollisionModeBruteForce);
				}
				else if (key == '2')
				{
					SUPERNOVA->getScene(0)->setCollisionMode(snCollisionMode::snECollisionMode_ST_SweepAndPrune);
					SCENEMGR->setCollisionMode(snCollisionMode::snECollisionMode_ST_SweepAndPrune);
				}
				else if (key == '3')
				{
					SUPERNOVA->getScene(0)->setCollisionMode(snCollisionMode::snECollisionMode_MT_SweepAndPrune);
					SCENEMGR->setCollisionMode(snCollisionMode::snECollisionMode_MT_SweepAndPrune);
				}
				return 0;
			}

			//Any other messages send to the default message handler as our application won't make use of them.
			default:
			{
				return DefWindowProc(hwnd, umsg, wparam, lparam);
			}
		}
	}

	void System::initializeWindows(int& screenWidth, int& screenHeight)
	{
		WNDCLASSEX wc;
		DEVMODE dmScreenSettings;
		int posX, posY;


		// Get an external pointer to this object.
		ApplicationHandle = this;

		// Get the instance of this application.
		m_hinstance = GetModuleHandle(NULL);

		// Give the application a name.
		m_applicationName = L"Engine";

		// Setup the windows class with default settings.
		wc.style = CS_HREDRAW | CS_VREDRAW | CS_OWNDC;
		wc.lpfnWndProc = WndProc;
		wc.cbClsExtra = 0;
		wc.cbWndExtra = 0;
		wc.hInstance = m_hinstance;
		wc.hIcon = LoadIcon(NULL, IDI_WINLOGO);
		wc.hIconSm = wc.hIcon;
		wc.hCursor = LoadCursor(NULL, IDC_ARROW);
		wc.hbrBackground = (HBRUSH)GetStockObject(BLACK_BRUSH);
		wc.lpszMenuName = NULL;
		wc.lpszClassName = m_applicationName;
		wc.cbSize = sizeof(WNDCLASSEX);

		// Register the window class.
		RegisterClassEx(&wc);

		// Determine the resolution of the clients desktop screen.
		screenWidth = GetSystemMetrics(SM_CXSCREEN);
		screenHeight = GetSystemMetrics(SM_CYSCREEN);

		// Setup the screen settings depending on whether it is running in full screen or in windowed mode.
		if (m_fullScreen)
		{
			// If full screen set the screen to maximum size of the users desktop and 32bit.
			memset(&dmScreenSettings, 0, sizeof(dmScreenSettings));
			dmScreenSettings.dmSize = sizeof(dmScreenSettings);
			dmScreenSettings.dmPelsWidth = (unsigned long)screenWidth;
			dmScreenSettings.dmPelsHeight = (unsigned long)screenHeight;
			dmScreenSettings.dmBitsPerPel = 32;
			dmScreenSettings.dmFields = DM_BITSPERPEL | DM_PELSWIDTH | DM_PELSHEIGHT;

			// Change the display settings to full screen.
			ChangeDisplaySettings(&dmScreenSettings, CDS_FULLSCREEN);

			// Set the position of the window to the top left corner.
			posX = posY = 0;
		}
		else
		{
			// If windowed then set it to 800x600 resolution.
			screenWidth = 1280;
			screenHeight = 720;

			// Place the window in the middle of the screen.
			posX = (GetSystemMetrics(SM_CXSCREEN) - screenWidth) / 2;
			posY = (GetSystemMetrics(SM_CYSCREEN) - screenHeight) / 2;
		}

		// Create the window with the screen settings and get the handle to it.
		m_hwnd = CreateWindowEx(WS_EX_APPWINDOW, m_applicationName, m_applicationName,
			/*WS_CLIPSIBLINGS | WS_CLIPCHILDREN | WS_POPUP |*/ WS_OVERLAPPEDWINDOW,
			posX, posY, screenWidth, screenHeight, NULL, NULL, m_hinstance, NULL);

		// Bring the window up on the screen and set it as main focus.
		ShowWindow(m_hwnd, SW_SHOW);
		SetForegroundWindow(m_hwnd);
		SetFocus(m_hwnd);

		// Hide the mouse cursor.
		ShowCursor(false);

		return;
	}

	void System::shutdownWindows()
	{
		// Show the mouse cursor.
		ShowCursor(true);

		// Fix the display settings if leaving full screen mode.
		if (m_fullScreen)
		{
			ChangeDisplaySettings(NULL, 0);
		}

		// Remove the window.
		DestroyWindow(m_hwnd);
		m_hwnd = NULL;

		// Remove the application instance.
		UnregisterClass(m_applicationName, m_hinstance);
		m_hinstance = NULL;

		// Release the pointer to this class.
		ApplicationHandle = NULL;

		return;
	}

	LRESULT CALLBACK WndProc(HWND hwnd, UINT umessage, WPARAM wparam, LPARAM lparam)
	{
		switch (umessage)
		{
			// Check if the window is being destroyed.
		case WM_DESTROY:
		{
						   PostQuitMessage(0);
						   return 0;
		}

			// Check if the window is being closed.
		case WM_CLOSE:
		{
						 PostQuitMessage(0);
						 return 0;
		}

			// All other messages pass to the message handler in the system class.
		default:
		{
				   return ApplicationHandle->messageHandler(hwnd, umessage, wparam, lparam);
		}
		}
	}
}