#ifndef D3D_H
#define D3D_H

//Include directx 11 library.
//#pragma comment(lib, "dxgi.lib")
//#pragma comment(lib, "d3d11.lib")
//#pragma comment(lib, "DXGI.lib")
//#pragma comment(lib, "d3dx11.lib")
//#pragma comment(lib, "d3dx10.lib")

//DirectX 11 includes.
#include <dxgi.h>
#include <d3dcommon.h>
#include <d3d11.h>
#include <DirectXMath.h>

using namespace DirectX;
namespace Devil
{

	class D3D
	{

	private:
		bool m_vsync_enabled;
		int m_videoCardMemory;
		char m_videoCardDescription[128];
		IDXGISwapChain* m_swapChain;
		ID3D11Device* m_device;
		ID3D11DeviceContext* m_deviceContext;
		ID3D11RenderTargetView* m_renderTargetView;
		ID3D11Texture2D* m_depthStencilBuffer;
		ID3D11DepthStencilState* m_depthStencilState;

		//Depth stencil state used for 2D drawing
		ID3D11DepthStencilState* m_depthDisabledStencilState;

		ID3D11DepthStencilView* m_depthStencilView;

		//Rasterizer state rendering in filled mode
		ID3D11RasterizerState* m_rasterStateFilled;

		//Rasterizer state rendering in wireframe mode
		ID3D11RasterizerState* m_rasterStateWireframe;

		XMMATRIX m_projectionMatrix;
		XMMATRIX m_worldMatrix;
		XMMATRIX m_orthoMatrix;

	public:
		D3D();
		virtual ~D3D();

		bool initialize(int screenWidth, int screenHeight, bool vsync, HWND hwnd, bool fullscreen, float screenDepth, float screenNear);
		void shutdown();

		//Prepare the scene to be rendered.
		//_clearColor is the color to use to clear the screen.
		void beginScene(const XMVECTORF32& _clearColor);

		void endScene();

		void TurnZBufferOn();
		void TurnZBufferOff();

		//Change the rasterizer state so all the next drawing calls will be in filled mode.
		void turnOnFillMode();

		//Change the rasterizer state so all the next drawing calls will be in wireframe mode
		void turnOnWireframeMode();


		ID3D11Device* getDevice();
		ID3D11DeviceContext* getDeviceContext();

		void getProjectionMatrix(XMMATRIX&);
		void getWorldMatrix(XMMATRIX&);
		void getOrthoMatrix(XMMATRIX&);

		void getVideoCardInfo(char*, int&);

		void* operator new(size_t _count)
		{
			return _aligned_malloc(_count, 16);
		}

		void operator delete(void* _p)
		{
			_aligned_free(_p);
		}

	private:
		//Create the rasterizer state to render in filled mode.
		bool createRasterStateFilled();

		//Create the rasterizer state to render in wireframe mode.
		bool createRasterStateWireframe();
	};
}

#endif
