#include "System.h"

#include "Input.h"
#include "Graphics.h"
#include "Camera.h"
#include "World.h"



#include "EffectProvider.h"

#include "EntitySphere.h"
#include "EntityBox.h"
#include "EntityPlan.h"
#include "EntityCamera.h"
#include "EntityFixedConstraint.h"

#include "snColliderSphere.h"
#include "snColliderPlan.h"

#include "snColliderBox.h"
#include "snScene.h"
#include "snActor.h"
#include "snFactory.h"
#include "snQuaternion.h"
#include "snFixedConstraint.h"


#include <string>
#include <Windowsx.h>

using namespace Supernova;

namespace Devil
{

	System::System() : m_fullScreen(false)
	{
		m_Input = 0;
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


		// Initialize the graphics object.
		result = GRAPHICS->initialize(screenWidth, screenHeight, m_hwnd, m_fullScreen);
		if (!result)
		{
			return false;
		}
		result = EFFECT->initialize();
		if (!result)
			return false;

		// Create the input object.  This object will be used to handle reading the keyboard input from the user.
		m_Input = new Input();
		if (!m_Input)
		{
			return false;
		}

		// Initialize the input object.
		m_Input->initialize(m_hwnd);

		//Initialize the world
		result = WORLD->initialize(m_Input);
		if (!result)
		{
			return false;
		}
		XMVECTOR cameraPosition = XMVectorSet(0, 0, -40, 1);
		XMVECTOR cameraLookAt = XMVectorSet(0, 0, 0, 1);
		XMVECTOR cameraUp = XMVectorSet(0, 1, 0, 0);
		WORLD->createCamera(cameraPosition, cameraLookAt, cameraUp);

		//initialize the physics
		result = SUPERNOVA->initialize();
		if (!result)
		{
			return false;
		}


		//createScene1();
		//createScene2();
		createSceneWithconstraint();

		//test case to debug the clipping plan bug.
		//createScene3();

		//createTower();

		return true;
	}

	void System::shutdown()
	{
		SUPERNOVA->clean();
		WORLD->shutdown();
		EFFECT->kill();
		GRAPHICS->shutdown();
		GRAPHICS->kill();

		// Release the input object.
		if (m_Input)
		{
			delete m_Input;
			m_Input = 0;
		}

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
				if (currentTime > m_deltaTime)
				{
					m_lastTick = currentClock.QuadPart;

					LARGE_INTEGER clockStart;
					QueryPerformanceCounter(&clockStart);
					m_physicScene->update(m_deltaTime * 0.001f);
					fpsCounterPhysics++;

					//update the world
					WORLD->update();

					// Check if the user pressed escape and wants to exit the application.
					if (m_Input->isKeyDown(VK_ESCAPE))
					{
						done = true;
					}

					if (m_Input->isKeyDown('C'))
					{
						WORLD->activateCollisionPoint();
					}
					if (m_Input->isKeyDown('V'))
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
					std::wstring strFPS = std::to_wstring(fpsCounter);

					std::wstring title = L"Supernova G=";
					title.append(strFPS);

					strFPS = std::to_wstring(fpsCounterPhysics);
					title.append(L" P=");
					title.append(strFPS);

					SetWindowText(m_hwnd, title.c_str());

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
			// Check if a key has been pressed on the keyboard.
			case WM_KEYDOWN:
			{
				// If a key is pressed send it to the input object so it can record that state.
				m_Input->keyDown((unsigned int)wparam);
				return 0;
			}

			//Check if a key has been released on the keyboard.
			case WM_KEYUP:
			{
				unsigned int key = (unsigned int)wparam;
				// If a key is released then send it to the input object so it can unset the state for that key.
				m_Input->keyUp(key);

				if (key == 32)
				{
					float width = 5;
					float height = 5;
					float depth = 5;
					/*float space = 1.5;*/
					/*float xOffset = -10;*/
					snVector4f pos(WORLD->getCamera()->getPosition());

					//create actor
					snActor* act = 0;
					int actorId = -1;
					m_physicScene->createActor(&act, actorId);

					act->setName("base");
					act->setMass(50);
					act->setPosition(pos);
					act->setIsKinematic(false);
					snVector4f linVel = WORLD->getCamera()->getLookAt() - WORLD->getCamera()->getPosition();
					linVel.normalize();
					linVel = linVel * 300;
					linVel.setW(0);
					act->setLinearVelocity(linVel);
					act->getPhysicMaterial().m_restitution = 1;
					act->getPhysicMaterial().m_friction = 0;

					//create collider
					snColliderBox* collider = 0;
					int colliderId = -1;
					act->createColliderBox(&collider, colliderId);

					collider->setSize(snVector4f(width, height, depth, 0));

					//initialize
					collider->initialize();
					act->initialize();

					EntityBox* box = WORLD->createBox(XMFLOAT3(width, height, depth), XMFLOAT4(0.8f, 1, 1, 1));
					box->setActor(act);
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

	void System::createScene1()
	{
		int sceneId = -1;
		SUPERNOVA->createScene(&m_physicScene, sceneId);
		m_physicScene->setGravity(snVector4f(0, -9.81f * 2, 0, 0));

		m_physicScene->setAngularSquaredSpeedThreshold(0.0001f);
		m_physicScene->setLinearSquaredSpeedThreshold(0.005f);
		m_physicScene->setSolverIterationCount(20);
		//set camera initial position
		WORLD->getCamera()->setPosition(XMVectorSet(25, 30, 50, 1));
		WORLD->getCamera()->setLookAt(XMVectorSet(15, 7, 0, 1));
		WORLD->getCamera()->setUp(XMVectorSet(0, 1, 0, 0));

		//WORLD->deactivateCollisionPoint();

		XMFLOAT4 color1(0.8f, 1, 1, 1);
		XMFLOAT4 color2(0.93f, 0.68f, 1, 1);
		XMFLOAT4 color3(1, 0.8f, 0.678f, 1);
		XMFLOAT4 color4(0.89f, 0.71f, 0.75f, 1);
		XMFLOAT4 color5(0.96f, 0.48f, 0.63f, 1);

		float groundHeight = 0;
		//ground
		{
			float width = 100;
			float height = 2;
			float depth = 100;

			//create actor
			snActor* act = 0;
			int actorId = -1;
			m_physicScene->createActor(&act, actorId);
			act->setName("ground");
			act->setMass(100);
			act->setPosition(snVector4f(0, 0, 0, 1));
			act->setIsKinematic(true);
			act->getPhysicMaterial().m_restitution = 0.f;
			act->getPhysicMaterial().m_friction = 0.f;

			//create collider
			snColliderBox* collider = 0;
			int colliderId = -1;
			act->createColliderBox(&collider, colliderId);
			collider->setSize(snVector4f(width, height, depth, 0));

			//initialize
			collider->initialize();
			act->initialize();

			EntityBox* kinematicBox = WORLD->createBox(XMFLOAT3(width, height, depth));
			kinematicBox->setActor(act);

			groundHeight += height * 0.5f;
		}
		//return;
		//first block on the ground
		float blockOneHeight = 0;
		{
			float width = 10;
			float height = 7;
			float depth = 10;

			snVector4f pos(0, groundHeight + height * 0.5f, 0, 1);

			//create actor
			snActor* act = 0;
			int actorId = -1;
			m_physicScene->createActor(&act, actorId);
			act->setName("base");
			act->setMass(200);
			act->setPosition(pos);
			act->setIsKinematic(false);
			act->getPhysicMaterial().m_restitution = 0.f;
			act->getPhysicMaterial().m_friction = 0.25f;

			//create collider
			snColliderBox* collider = 0;
			int colliderId = -1;
			act->createColliderBox(&collider, colliderId);
			collider->setSize(snVector4f(width, height, depth, 0));

			//initialize
			collider->initialize();
			act->initialize();

			EntityBox* box = WORLD->createBox(XMFLOAT3(width, height, depth), color1);
			box->setActor(act);

			blockOneHeight = pos.getY() + height * 0.5f;
		}
		//return;
		//platform
		float platformHeight = 0;
		{
			float width = 15;
			float height = 0.5;
			float depth = 3;
			snVector4f pos(width * 0.5f, blockOneHeight + height * 0.5f + 0, 0, 1);

			//create actor
			snActor* act = 0;
			int actorId = -1;
			m_physicScene->createActor(&act, actorId);
			act->setName("platform");
			act->setMass(100);
			act->setPosition(pos);
			act->setIsKinematic(false);
			act->getPhysicMaterial().m_restitution = 0.f;
			act->getPhysicMaterial().m_friction = 1.f;

			//create collider
			snColliderBox* collider = 0;
			int colliderId = -1;
			act->createColliderBox(&collider, colliderId);
			collider->setSize(snVector4f(width, height, depth, 0));

			//initialize
			collider->initialize();
			act->initialize();

			EntityBox* box = WORLD->createBox(XMFLOAT3(width, height, depth), color2);
			box->setActor(act);

			platformHeight = pos.getY() + height * 0.5f;
		}
		//return;
		//second block
		float blockTwoHeight = 0;
		{
			float width = 3;
			float height = 3;
			float depth = 3;
			snVector4f pos(2, platformHeight + height * 0.5f, 0, 1);

			//create actor
			snActor* act = 0;
			int actorId = -1;
			m_physicScene->createActor(&act, actorId);
			act->setName("two");
			act->setMass(100);
			act->setPosition(pos);
			act->setIsKinematic(false);
			act->getPhysicMaterial().m_restitution = 0.f;
			act->getPhysicMaterial().m_friction = 1.f;

			//create collider
			snColliderBox* collider = 0;
			int colliderId = -1;
			act->createColliderBox(&collider, colliderId);
			collider->setSize(snVector4f(width, height, depth, 0));

			//initialize
			collider->initialize();
			act->initialize();

			EntityBox* box = WORLD->createBox(XMFLOAT3(width, height, depth), color3);
			box->setActor(act);

			blockTwoHeight = pos.getY() + height * 0.5f;
		}
		//return;
		float blockThreeHeight = 0;
		{
			float width = 3;
			float height = 3;
			float depth = 3;
			snVector4f pos(2, blockTwoHeight + height * 0.5f, 0, 1);

			//create actor
			snActor* act = 0;
			int actorId = -1;
			m_physicScene->createActor(&act, actorId);
			act->setName("three");
			act->setMass(100);
			act->setPosition(pos);
			act->setIsKinematic(false);
			act->getPhysicMaterial().m_restitution = 0.f;
			act->getPhysicMaterial().m_friction = 1.f;

			//create collider
			snColliderBox* collider = 0;
			int colliderId = -1;
			act->createColliderBox(&collider, colliderId);
			collider->setSize(snVector4f(width, height, depth, 0));

			//initialize
			collider->initialize();
			act->initialize();

			EntityBox* box = WORLD->createBox(XMFLOAT3(width, height, depth), color5);
			box->setActor(act);

			blockThreeHeight = pos.getY() + height * 0.5f;
		}

		//return;
		//dynamic
		{
			float width = 3;
			float height = 3;
			float depth = 3;
			snVector4f pos(11, platformHeight + height * 0.5f + 25, 0, 1);

			//create actor
			snActor* act = 0;
			int actorId = -1;
			m_physicScene->createActor(&act, actorId);
			act->setName("dymnamic");
			act->setMass(500);
			act->setPosition(pos);
			act->setIsKinematic(false);
			act->getPhysicMaterial().m_restitution = 0.f;
			act->getPhysicMaterial().m_friction = 1.f;

			//create collider
			snColliderBox* collider = 0;
			int colliderId = -1;
			act->createColliderBox(&collider, colliderId);
			collider->setSize(snVector4f(width, height, depth, 0));

			//initialize
			collider->initialize();
			act->initialize();

			EntityBox* box = WORLD->createBox(XMFLOAT3(width, height, depth), color4);
			box->setActor(act);

			blockTwoHeight = pos.getY() + height * 0.5f;
		}

	}

	void System::createScene2()
	{
		int sceneId = -1;
		SUPERNOVA->createScene(&m_physicScene, sceneId);
		m_physicScene->setGravity(snVector4f(0, -9.81f * 2, 0, 0));
		m_physicScene->setSolverIterationCount(60);
		m_physicScene->setLinearSquaredSpeedThreshold(0.006f);
		m_physicScene->setAngularSquaredSpeedThreshold(0.0005f);

		//set camera initial position
		WORLD->getCamera()->setPosition(XMVectorSet(70, 50, 100, 1));
		WORLD->getCamera()->setLookAt(XMVectorSet(15, 15, 0, 1));
		WORLD->getCamera()->setUp(XMVectorSet(0, 1, 0, 0));

		WORLD->deactivateCollisionPoint();

		XMFLOAT4 colors[5];
		colors[0] = XMFLOAT4(0.8f, 1, 1, 1);
		colors[1] = XMFLOAT4(0.93f, 0.68f, 1, 1);
		colors[2] = XMFLOAT4(1, 0.8f, 0.678f, 1);
		colors[3] = XMFLOAT4(0.89f, 0.71f, 0.75f, 1);
		colors[4] = XMFLOAT4(0.96f, 0.48f, 0.63f, 1);

		float groundHeight = 0;
		//ground
		{
			float width = 200;
			float height = 2;
			float depth = 200;

			//create actor
			snActor* act = 0;
			int actorId = -1;
			m_physicScene->createStaticActor(&act, actorId);

			act->setName("ground");
			act->setMass(100);
			act->setPosition(snVector4f(0, 0, 0, 1));
			//act->setIsKinematic(true);
			act->getPhysicMaterial().m_restitution = 1;
			act->getPhysicMaterial().m_friction = 0.8f;

			//create collider
			snColliderBox* collider = 0;
			int colliderId = -1;
			act->createColliderBox(&collider, colliderId);

			collider->setSize(snVector4f(width, height, depth, 0));

			//initialize
			collider->initialize();
			act->initialize();
			
			//create the world entity
			EntityBox* kinematicBox = WORLD->createBox(XMFLOAT3(width, height, depth));
			kinematicBox->setActor(act);

			groundHeight += height * 0.5f;
		}

		//back wall
		{
			float width = 200;
			float height = 200;
			float depth = 5;

			//create actor
			snActor* act = 0;
			int actorId = -1;
			m_physicScene->createStaticActor(&act, actorId);

			act->setName("back");
			act->setMass(100);
			act->setPosition(snVector4f(0, 101, -80, 1));
			//act->setIsKinematic(true);
			act->getPhysicMaterial().m_restitution = 1;
			act->getPhysicMaterial().m_friction = 1;

			//create collider
			snColliderBox* collider = 0;
			int colliderId = -1;
			act->createColliderBox(&collider, colliderId);

			collider->setSize(snVector4f(width, height, depth, 0));

			//initialize
			collider->initialize();
			act->initialize();

			//create world entity
			EntityBox* kinematicBox = WORLD->createBox(XMFLOAT3(width, height, depth));
			kinematicBox->setActor(act);
		}

		const int MAX_ROW = 14;
		for (int row = MAX_ROW; row > 0; --row)
		{
			for (int i = 0; i < row; ++i)
			{
				float width = 5;
				float height = 5;
				float depth = 5;
				float space = 1.5;
				float xOffset = -10;
				snVector4f pos((MAX_ROW - row) * width * 0.5f + (width + space) * i + xOffset, groundHeight + height * 0.5f + height * (MAX_ROW - row), 0, 1);

				//create actor
				snActor* act = 0;
				int actorId = -1;
				m_physicScene->createActor(&act, actorId);

				act->setName("base_" + std::to_string(row) + "_" + std::to_string(i));
				act->setMass(50);
				act->setPosition(pos);
				act->setIsKinematic(false);
				act->getPhysicMaterial().m_restitution = 0;
				act->getPhysicMaterial().m_friction = 1;

				//create collider
				snColliderBox* collider = 0;
				int colliderId = -1;
				act->createColliderBox(&collider, colliderId);

				collider->setSize(snVector4f(width, height, depth, 0));

				//initialize
				collider->initialize();
				act->initialize();

				EntityBox* box = WORLD->createBox(XMFLOAT3(width, height, depth), colors[(i + row) % 5]);
				box->setActor(act);
			}
		}
		return;
		{
			float width = 5;
			float height = 5;
			float depth = 5;
			snVector4f pos(10, 45, 50, 1);

			//create actor
			snActor* act = 0;
			int actorId = -1;
			m_physicScene->createActor(&act, actorId);
			act->setName("dynamic");
			act->setMass(500);
			act->setPosition(pos);
			act->setLinearVelocity(snVector4f(0, -50, -80, 0));
			act->getPhysicMaterial().m_restitution = 0;
			act->getPhysicMaterial().m_friction = 1;
			act->setIsKinematic(false);

			//create collider
			snColliderBox* collider = 0;
			int colliderId = -1;
			act->createColliderBox(&collider, colliderId);

			collider->setSize(snVector4f(width, height, depth, 0));

			//initialize
			collider->initialize();
			act->initialize();

			EntityBox* box = WORLD->createBox(XMFLOAT3(width, height, depth), colors[4]);
			box->setActor(act);
		}
	}

	void System::createScene3()
	{
		int sceneId = -1;
		SUPERNOVA->createScene(&m_physicScene, sceneId);
		m_physicScene->setSolverIterationCount(20);
		m_physicScene->setBeta(0.2f);
		m_physicScene->setGravity(snVector4f(0, -9.81f * 0.5f, 0, 0));

		//set camera initial position
		WORLD->getCamera()->setPosition(XMVectorSet(25, 30, 50, 1));
		WORLD->getCamera()->setLookAt(XMVectorSet(15, 7, 0, 1));
		WORLD->getCamera()->setUp(XMVectorSet(0, 1, 0, 0));

		//WORLD->deactivateCollisionPoint();

		XMFLOAT4 color1(0.8f, 1, 1, 1);
		XMFLOAT4 color2(0.93f, 0.68f, 1, 1);
		XMFLOAT4 color3(1, 0.8f, 0.678f, 1);
		XMFLOAT4 color4(0.89f, 0.71f, 0.75f, 1);
		XMFLOAT4 color5(0.96f, 0.48f, 0.63f, 1);

		float groundHeight = 0;
		//ground
		{
			float width = 1000;
			float height = 2;
			float depth = 1000;

			//create actor
			snActor* act = 0;
			int actorId = -1;
			m_physicScene->createActor(&act, actorId);
			act->setName("ground");
			act->setMass(100);
			act->setPosition(snVector4f(0, 0, 0, 1));
			act->setIsKinematic(true);
			act->getPhysicMaterial().m_restitution = 1.f;
			act->getPhysicMaterial().m_friction = 0.f;

			//create collider
			snColliderBox* collider = 0;
			int colliderId = -1;
			act->createColliderBox(&collider, colliderId);
			collider->setSize(snVector4f(width, height, depth, 0));

			//initialize
			collider->initialize();
			act->initialize();

			EntityBox* kinematicBox = WORLD->createBox(XMFLOAT3(width, height, depth));
			kinematicBox->setActor(act);

			groundHeight += height * 0.5f;
		}

		//back wall
		//{
		//	float width = 200;
		//	float height = 200;
		//	float depth = 2;

		//	//create actor
		//	snActor* act = 0;
		//	int actorId = -1;
		//	m_physicScene->createActor(&act, actorId);

		//	act->setName("back");
		//	act->setMass(100);
		//	act->setPosition(snVector4f(0, 101, -80, 1));
		//	act->setIsKinematic(true);
		//	act->getPhysicMaterial().m_restitution = 1;
		//	act->getPhysicMaterial().m_friction = 1;

		//	//create collider
		//	snColliderBox* collider = 0;
		//	int colliderId = -1;
		//	act->createColliderBox(&collider, colliderId);

		//	collider->setSize(snVector4f(width, height, depth, 0));

		//	//initialize
		//	collider->initialize();
		//	act->initialize();

		//	//create world entity
		//	EntityBox* kinematicBox = WORLD->createBox(XMFLOAT3(width, height, depth));
		//	kinematicBox->setActor(act);
		//}

		//platform
		float platformHeight = 0;
		{
			float width = 7;
			float height = 1;
			float depth = 3;
			snVector4f pos(width * 0.5f, 20 + height * 0.5f + 0, 0, 1);

			//create actor
			snActor* act = 0;
			int actorId = -1;
			m_physicScene->createActor(&act, actorId);
			act->setName("platform");
			act->setMass(1);
			act->setPosition(pos);
			float angle = 3.14f * 0.25f;
			act->setOrientationQuaternion(snQuaternionFromEuler(angle, angle, 0));
			//act->setAngularVelocity(snVector4f(0, 10, 20, 0));
			act->setIsKinematic(false);
			act->getPhysicMaterial().m_restitution = 0.f;
			act->getPhysicMaterial().m_friction = 1.f;

			//create collider
			snColliderBox* collider = 0;
			int colliderId = -1;
			act->createColliderBox(&collider, colliderId);
			collider->setSize(snVector4f(width, height, depth, 0));

			//initialize
			collider->initialize();
			act->initialize();

			EntityBox* box = WORLD->createBox(XMFLOAT3(width, height, depth), color2);
			box->setActor(act);

			platformHeight = pos.getY() + height * 0.5f;
		}
		return;
		//dynamic
		{
			float width = 20;
			float height = 1;
			float depth = 3;
			snVector4f pos(width * 0.5f, 30 + height * 0.5f + 0, 0, 1);

			//create actor
			snActor* act = 0;
			int actorId = -1;
			m_physicScene->createActor(&act, actorId);
			act->setName("dymnamic");
			act->setMass(1);
			act->setPosition(pos);
			act->setOrientationQuaternion(snQuaternionFromEuler(0, 3.14f * 0.5f, 0));
			act->setIsKinematic(false);
			act->getPhysicMaterial().m_restitution = 0.f;
			act->getPhysicMaterial().m_friction = 1.f;

			//create collider
			snColliderBox* collider = 0;
			int colliderId = -1;
			act->createColliderBox(&collider, colliderId);
			collider->setSize(snVector4f(width, height, depth, 0));

			//initialize
			collider->initialize();
			act->initialize();

			EntityBox* box = WORLD->createBox(XMFLOAT3(width, height, depth), color4);
			box->setActor(act);

			//blockTwoHeight = pos.getY() + height * 0.5f;
		}

	}

	void System::createSceneWithconstraint()
	{
		int sceneId = -1;
		SUPERNOVA->createScene(&m_physicScene, sceneId);
		m_physicScene->setGravity(snVector4f(0, -9.81f * 4, 0, 0));
		m_physicScene->setSolverIterationCount(4);
		m_physicScene->setLinearSquaredSpeedThreshold(0.006f);
		m_physicScene->setAngularSquaredSpeedThreshold(0.001f);

		//set camera initial position
		WORLD->getCamera()->setPosition(XMVectorSet(0, 50, 100, 1));
		WORLD->getCamera()->setLookAt(XMVectorSet(15, 15, 0, 1));
		WORLD->getCamera()->setUp(XMVectorSet(0, 1, 0, 0));

		//WORLD->deactivateCollisionPoint();
		WORLD->activateCollisionPoint();

		XMFLOAT4 colors[5];
		colors[0] = XMFLOAT4(0.8f, 1, 1, 1);
		colors[1] = XMFLOAT4(0.93f, 0.68f, 1, 1);
		colors[2] = XMFLOAT4(1, 0.8f, 0.678f, 1);
		colors[3] = XMFLOAT4(0.89f, 0.71f, 0.75f, 1);
		colors[4] = XMFLOAT4(0.96f, 0.48f, 0.63f, 1);

		float groundHeight = 0;
		//ground
		{
			float width = 200;
			float height = 2;
			float depth = 200;

			//create actor
			snActor* act = 0;
			int actorId = -1;
			m_physicScene->createStaticActor(&act, actorId);

			act->setName("ground");
			act->setMass(100);
			act->setPosition(snVector4f(0, 0, 0, 1));
			act->getPhysicMaterial().m_restitution = 1;
			act->getPhysicMaterial().m_friction = 0.8f;

			//create collider
			snColliderBox* collider = 0;
			int colliderId = -1;
			act->createColliderBox(&collider, colliderId);

			collider->setSize(snVector4f(width, height, depth, 0));

			//initialize
			collider->initialize();
			act->initialize();

			//create the world entity
			EntityBox* kinematicBox = WORLD->createBox(XMFLOAT3(width, height, depth));
			kinematicBox->setActor(act);

			groundHeight += height * 0.5f;
		}

		//back wall
		{
			float width = 200;
			float height = 200;
			float depth = 5;

			//create actor
			snActor* act = 0;
			int actorId = -1;
			m_physicScene->createStaticActor(&act, actorId);

			act->setName("back");
			act->setMass(100);
			act->setPosition(snVector4f(0, 101, -80, 1));
			//act->setIsKinematic(true);
			act->getPhysicMaterial().m_restitution = 1;
			act->getPhysicMaterial().m_friction = 1;

			//create collider
			snColliderBox* collider = 0;
			int colliderId = -1;
			act->createColliderBox(&collider, colliderId);

			collider->setSize(snVector4f(width, height, depth, 0));

			//initialize
			collider->initialize();
			act->initialize();

			//create world entity
			EntityBox* kinematicBox = WORLD->createBox(XMFLOAT3(width, height, depth));
			kinematicBox->setActor(act);
		}
		snActor* act1 = 0;
		{
			float width = 5;
			float height = 5;
			float depth = 5;
			snVector4f pos(10, 35, 0, 1);

			//create actor
			snActor* act = 0;
			int actorId = -1;
			m_physicScene->createActor(&act, actorId);
			act->setName("d0");
			act->setMass(50);
			act->setPosition(pos);
			act->getPhysicMaterial().m_restitution = 0;
			act->getPhysicMaterial().m_friction = 1;
			act->setIsKinematic(false);
			act->setLinearDamping(20);
			act->setAngularDamping(20);
			act1 = act;

			//create collider
			snColliderBox* collider = 0;
			int colliderId = -1;
			act->createColliderBox(&collider, colliderId);

			collider->setSize(snVector4f(width, height, depth, 0));

			//initialize
			collider->initialize();
			act->initialize();

			EntityBox* box = WORLD->createBox(XMFLOAT3(width, height, depth), colors[4]);
			box->setActor(act);

			//create constraints
			int constraintId = m_physicScene->createFixedConstraint(act, pos + snVector4f(-7, 10, 0, 0), 10, m_deltaTime * 0.001f);
			snIConstraint* c = m_physicScene->getConstraint(constraintId);
			snFixedConstraint* constraint = static_cast<snFixedConstraint*>(c);
			WORLD->createFixedConstraint(constraint);
		}
		
		float width = 5;
		float height = 5;
		float depth = 5;
		snVector4f pos(-10, 35, 0, 1);

		//create actor
		snActor* act = 0;
		int actorId = -1;
		m_physicScene->createActor(&act, actorId);
		act->setName("d1");
		act->setMass(50);
		act->setPosition(pos);
		act->getPhysicMaterial().m_restitution = 0;
		act->getPhysicMaterial().m_friction = 1;
		act->setIsKinematic(false);

		//create collider
		snColliderBox* collider = 0;
		int colliderId = -1;
		act->createColliderBox(&collider, colliderId);

		collider->setSize(snVector4f(width, height, depth, 0));

		//initialize
		collider->initialize();
		act->initialize();

		EntityBox* box = WORLD->createBox(XMFLOAT3(width, height, depth), colors[4]);
		box->setActor(act);	

		//int p2pcid = m_physicScene->createPointToPointConstraint(act1, snVector4f(-10, 0, 0, 1), act, snVector4f(10, 0, 0, 1));
	}

	snVector4f System::createTowerLevel(const snVector4f& _origin)
	{
		//colors
		const int colorCount = 5;
		XMFLOAT4 colors[colorCount];
		colors[0] = XMFLOAT4(0.8f, 1, 1, 1);
		colors[1] = XMFLOAT4(0.93f, 0.68f, 1, 1);
		colors[2] = XMFLOAT4(1, 0.8f, 0.678f, 1);
		colors[3] = XMFLOAT4(0.89f, 0.71f, 0.75f, 1);
		colors[4] = XMFLOAT4(0.96f, 0.48f, 0.63f, 1);

		float towerWidth = 20;
		float pillarHeight = 5;
		float pillarWidth = 2;
		float pillarDepth = 2;

		const int pillarCount = 4;
		snVector4f pillarPosition[pillarCount];

		float halfTowerWidth = towerWidth * 0.5f;
		float halfPillarHeight = pillarHeight * 0.5f;
		pillarPosition[0] = snVector4f(halfTowerWidth, halfPillarHeight, halfTowerWidth, 0);
		pillarPosition[1] = snVector4f(-halfTowerWidth, halfPillarHeight, halfTowerWidth, 0);
		pillarPosition[2] = snVector4f(-halfTowerWidth, halfPillarHeight, -halfTowerWidth, 0);
		pillarPosition[3] = snVector4f(halfTowerWidth, halfPillarHeight, -halfTowerWidth, 0);

		//create pillars
		for (int i = 0; i < pillarCount; ++i)
		{
			//create actor
			snActor* act = 0;
			int actorId = -1;
			m_physicScene->createActor(&act, actorId);

			snVector4f pos = pillarPosition[i] + _origin;
			act->setName("pillar1");
			act->setMass(50);
			act->setPosition(pos);
			act->setIsKinematic(false);
			act->getPhysicMaterial().m_restitution = 0;
			act->getPhysicMaterial().m_friction = 1;

			//create collider
			snColliderBox* collider = 0;
			int colliderId = -1;
			act->createColliderBox(&collider, colliderId);

			collider->setSize(snVector4f(pillarWidth, pillarHeight, pillarDepth, 0));

			//initialize
			collider->initialize();
			act->initialize();

			EntityBox* box = WORLD->createBox(XMFLOAT3(pillarWidth, pillarHeight, pillarDepth), colors[actorId % colorCount]);
			box->setActor(act);
		}

		float bedWidth = towerWidth + 2 * pillarWidth;
		float bedDepth = pillarDepth * 2;
		float bedHeight = 1;

		float firstBedHeight = pillarHeight + bedHeight * 0.5f;
		float secondBedHeight = firstBedHeight + bedHeight;

		snVector4f bedPosition[4];
		bedPosition[0] = snVector4f(0, firstBedHeight, halfTowerWidth, 0);
		bedPosition[1] = snVector4f(0, firstBedHeight, -halfTowerWidth, 0);
		bedPosition[2] = snVector4f(halfTowerWidth, secondBedHeight, 0, 0);
		bedPosition[3] = snVector4f(-halfTowerWidth, secondBedHeight, 0, 0);

		snVector4f bedSize[4];
		bedSize[0] = snVector4f(bedWidth, bedHeight, bedDepth, 0);
		bedSize[1] = snVector4f(bedWidth, bedHeight, bedDepth, 0);
		bedSize[2] = snVector4f(bedDepth, bedHeight, bedWidth, 0);
		bedSize[3] = snVector4f(bedDepth, bedHeight, bedWidth, 0);

		//create bed between pillars
		for (int i = 0; i < pillarCount; ++i)
		{
			//create actor
			snActor* act = 0;
			int actorId = -1;
			m_physicScene->createActor(&act, actorId);

			snVector4f pos = bedPosition[i] + _origin;
			act->setName("bed");
			act->setMass(50);
			act->setPosition(pos);
			act->setIsKinematic(false);
			act->getPhysicMaterial().m_restitution = 0;
			act->getPhysicMaterial().m_friction = 0.9f;

			//create collider
			snColliderBox* collider = 0;
			int colliderId = -1;
			act->createColliderBox(&collider, colliderId);

			collider->setSize(bedSize[i]);

			//initialize
			collider->initialize();
			act->initialize();

			EntityBox* box = WORLD->createBox(XMFLOAT3(bedSize[i].getX(), bedSize[i].getY(), bedSize[i].getZ()), colors[actorId % colorCount]);
			box->setActor(act);
		}

		return _origin + snVector4f(0, pillarHeight + 2* bedHeight, 0, 0);
	}

	void System::createTower()
	{
		int sceneId = -1;
		SUPERNOVA->createScene(&m_physicScene, sceneId);
		m_physicScene->setSolverIterationCount(45);
		m_physicScene->setLinearSquaredSpeedThreshold(0.006f);
		m_physicScene->setAngularSquaredSpeedThreshold(0.001f);

		//set camera initial position
		WORLD->getCamera()->setPosition(XMVectorSet(50, 50, 70, 1));
		WORLD->getCamera()->setLookAt(XMVectorSet(0, 7, 0, 1));
		WORLD->getCamera()->setUp(XMVectorSet(0, 1, 0, 0));

		WORLD->deactivateCollisionPoint();

		
		float groundHeight = 0;
		//ground
		{
			float width = 2000;
			float height = 2;
			float depth = 2000;

			//create actor
			snActor* act = 0;
			int actorId = -1;
			m_physicScene->createActor(&act, actorId);

			act->setName("ground");
			act->setMass(100);
			act->setPosition(snVector4f(0, 0, 0, 1));
			act->setIsKinematic(true);
			act->getPhysicMaterial().m_restitution = 0;
			act->getPhysicMaterial().m_friction = 0.8f;

			//create collider
			snColliderBox* collider = 0;
			int colliderId = -1;
			act->createColliderBox(&collider, colliderId);

			collider->setSize(snVector4f(width, height, depth, 0));

			//initialize
			collider->initialize();
			act->initialize();

			//create the world entity
			EntityBox* kinematicBox = WORLD->createBox(XMFLOAT3(width, height, depth));
			kinematicBox->setActor(act);

			groundHeight += height * 0.5f;
		}

		//back wall
		{
			float width = 200;
			float height = 200;
			float depth = 2;

			//create actor
			snActor* act = 0;
			int actorId = -1;
			m_physicScene->createActor(&act, actorId);

			act->setName("back");
			act->setMass(100);
			act->setPosition(snVector4f(0, 101, -80, 1));
			act->setIsKinematic(true);
			act->getPhysicMaterial().m_restitution = 1;
			act->getPhysicMaterial().m_friction = 1;

			//create collider
			snColliderBox* collider = 0;
			int colliderId = -1;
			act->createColliderBox(&collider, colliderId);

			collider->setSize(snVector4f(width, height, depth, 0));

			//initialize
			collider->initialize();
			act->initialize();

			//create world entity
			EntityBox* kinematicBox = WORLD->createBox(XMFLOAT3(width, height, depth));
			kinematicBox->setActor(act);
		}

		//tower
		snVector4f origin(0, groundHeight, 0, 1);

		int levelCount = 5;
		for (int i = 0; i < levelCount; ++i)
		{
			origin = createTowerLevel(origin);
		}
		return;
		//projectile
		{
			float width = 2;
			float height = 2;
			float depth = 2;

			//create actor
			snActor* act = 0;
			int actorId = -1;
			m_physicScene->createActor(&act, actorId);

			act->setName("ground");
			act->setMass(500);
			act->setPosition(snVector4f(-0, 20, 50, 1));
			act->setLinearVelocity(snVector4f(15, 0, -50, 0));
			act->setIsKinematic(false);
			act->getPhysicMaterial().m_restitution = 0;
			act->getPhysicMaterial().m_friction = 0.8f;

			//create collider
			snColliderBox* collider = 0;
			int colliderId = -1;
			act->createColliderBox(&collider, colliderId);

			collider->setSize(snVector4f(width, height, depth, 0));

			//initialize
			collider->initialize();
			act->initialize();

			//create the world entity
			EntityBox* kinematicBox = WORLD->createBox(XMFLOAT3(width, height, depth), XMFLOAT4(1, 0, 0, 1));
			kinematicBox->setActor(act);
		}
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