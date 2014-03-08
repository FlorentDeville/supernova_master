#ifndef SYSTEM_H
#define SYSTEM_H

//#define WIN32_LEAN_AND_MEAN
#include <Windows.h>

#include "snScene.h"
using Supernova::snScene;

#include "snVector4f.h"
using Supernova::snVector4f;

namespace Devil
{
	class Input;
	class Graphics;
	class EntitySphere;
	
	__declspec(align(16)) class System
	{
	private:
		LPCWSTR m_applicationName;
		HINSTANCE m_hinstance;
		HWND m_hwnd;

		Input* m_Input;

		EntitySphere* m_Sphere;

		int TICK = 60;

		unsigned long m_deltaTime;
		unsigned long m_lastTick;

		//Physics scene.
		snScene* m_physicScene;

		//Flag to indicate if the window is full screen or not.
		bool m_fullScreen;

	public:
		System();
		virtual ~System();

		bool initialize(bool _fullScreen);
		void run();
		void shutdown();
		LRESULT CALLBACK messageHandler(HWND, UINT, WPARAM, LPARAM);

		void* operator new(size_t _count)
		{
			return _aligned_malloc(_count, 16);
		}

		void operator delete(void* _p)
		{
			_aligned_free(_p);
		}

	private:
		void initializeWindows(int&, int&);
		void shutdownWindows();

		void createScene1();
		void createScene2();
		void createScene3();
		void createSceneWithconstraint();

		snVector4f createTowerLevel(const snVector4f& _origin);
		void createTower();
	};

	static LRESULT CALLBACK WndProc(HWND, UINT, WPARAM, LPARAM);

	static System* ApplicationHandle = 0;
}
#endif //SYSTEM_H
