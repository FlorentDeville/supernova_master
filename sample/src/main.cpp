#define WIN32_LEAN_AND_MEAN
#include <Windows.h>

//#include "Vector4f.h"
//#include "Matrix44f.h"


//#include "ColliderBox.h"
//#include "ColliderSphere.h"
//#include "Collision.h"

//#include <d3d11.h>
//#include "ColliderPlan.h"

#include <DirectXMath.h>
using namespace DirectX;

#include "System.h"

#include "LemkeAlgorithm.h"
#include "PGSSolver.h"
#include "MatrixX.h"
#include "VectorX.h"

using namespace Devil;

int CALLBACK WinMain(HINSTANCE /*hInstance*/, HINSTANCE /*hPrevInstance*/, LPSTR /*lpCmdLine*/, int /*nCmdShow*/)
{
	System* GlobalSystem;
	bool result;


	// Create the system object.
	GlobalSystem = new System();
	if (!GlobalSystem)
	{
		return 0;
	}

	// Initialize and run the system object.
	result = GlobalSystem->initialize();
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