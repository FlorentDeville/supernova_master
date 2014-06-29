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
	snOBB box1(snVec4Set(5, 3.5, 5, 0));
	box1.initialize();
	snMatrix44f transform;
	transform.createTranslation(snVec4Set(0, 3.5, 0, 1));
	box1.setTransform(transform);

	snOBB box2(snVec4Set(1.5, 1.5, 1.5, 0));
	box2.initialize();
	transform.createTranslation(snVec4Set(2, 3.5, 0, 1));
	box2.setTransform(transform);

	bool resGJK1 = snGJK::gjkIntersect<snOBB, snOBB>(box1, box2);

	snGJK gjk;
	snCollisionResult resGJK2 = gjk.queryIntersection(box1, box2);

	snVec normal = snVec4Set(0);
	bool resSAT = snSAT::queryIntersection<snOBB, snOBB>(box1, box2, normal);

#endif
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