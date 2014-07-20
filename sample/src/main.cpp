#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#include "System.h"

#ifdef _DEBUG
//needed for memory leak detection. Do not change the order.
#include <stdlib.h>
#include <crtdbg.h>

#include "snOBB.h"
#include "snGJK.h"
#include "snSAT.h"
#include "snVec.h"
#include "snMatrix44f.h"
using namespace Supernova::Vector;
using namespace Supernova;
#endif //_DEBUG

using namespace Devil;

int CALLBACK WinMain(HINSTANCE /*hInstance*/, HINSTANCE /*hPrevInstance*/, LPSTR /*lpCmdLine*/, int /*nCmdShow*/)
{
#ifdef _DEBUG
	//enable memory leak detection
	_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);
#endif //_DEBUG

	System* GlobalSystem;
	bool result;


	// Create the system object.
	GlobalSystem = new System();
	if (!GlobalSystem)
	{
		return 0;
	}

	// Initialize and run the system object.
	bool fullScreen = false;
	result = GlobalSystem->initialize(fullScreen);
	if (result)
	{
		GlobalSystem->run();
	}

	// Shutdown and release the system object.
	GlobalSystem->shutdown();
	delete GlobalSystem;
	GlobalSystem = 0;

	return 0;
}