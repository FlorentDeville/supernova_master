#ifndef SYSTEM_H
#define SYSTEM_H

//#define WIN32_LEAN_AND_MEAN
#include <Windows.h>

#include "snScene.h"
using Supernova::snScene;

#include "snVector4f-inl.h"
using Supernova::snVector4f;

namespace Devil
{
	class Graphics;
	class EntitySphere;
	
	class System
	{
	private:
		LPCWSTR m_applicationName;
		HINSTANCE m_hinstance;
		HWND m_hwnd;

		EntitySphere* m_Sphere;

		static const int TICK = 60;

		unsigned long m_deltaTime;
		long long m_lastTick;

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

	private:
		void initializeWindows(int&, int&);
		void shutdownWindows();
	};

	static LRESULT CALLBACK WndProc(HWND, UINT, WPARAM, LPARAM);

	static System* ApplicationHandle = 0;
}
#endif //SYSTEM_H
