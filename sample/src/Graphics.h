#ifndef GRAPHICS_H
#define GRAPHICS_H

#define WIN32_LEAN_AND_MEAN
#include <Windows.h>

#include <vector>
#include "IGfxEntity.h"
#include <DirectXColors.h>

namespace Devil
{
	class D3D;
	class Camera;
	class Model;
	class ColorShader;
	class TextureShader;
	class LightShader;
	class Light;

	class GfxEntitySphere;
	class GfxEntityBox;
	class GfxEntityPlan;

	//const bool FULL_SCREEN = false;
	const bool VSYNC_ENABLED = false;
	const float SCREEN_DEPTH = 1000.0f;
	const float SCREEN_NEAR = 0.1f;

	class Graphics
	{

	private:

		static Graphics* m_Instance;

		D3D* m_D3D;

		Camera* m_Camera;
		Model* m_Model;
		ColorShader* m_ColorShader;
		TextureShader* m_TextureShader;
		LightShader* m_LightShader;
		Light* m_Light;

		std::vector<IGfxEntity*> m_entityList;

		//Width of the screen in pixels
		int m_screenWidth;

		//Height of the screen in pixels.
		int m_screenHeight;

		//Define the color to use to clear the screen.
		XMVECTORF32 m_clearScreenColor;

	public:
		static Graphics* getInstance();

		
		~Graphics();

		bool initialize(int _screenWidth, int _screenHeight, HWND _hwnd, bool _fullScreen);
		void shutdown();
		
		void BeginRender();
		void EndRender();

		GfxEntitySphere* createSphere(float, const XMVECTOR& _color = Colors::White);
		GfxEntityBox* createBox(const XMFLOAT3&, const XMFLOAT4&);
		GfxEntityPlan* createPlan(const XMFLOAT2&, const XMFLOAT4&);

		D3D* getDirectXWrapper();
		Camera* getCamera();

		//Get the width of the screen in pixel.
		int getScreenWidth() const;

		//Get the height of the screen in pixel.
		int getScreenHeight() const;

		//Set the color to use to clear the screen.
		void setClearScreenColor(const XMVECTORF32& _color);

		//Delete all the graphics entity created
		void clear();

	private:
		Graphics();
	};

#define GRAPHICS Graphics::getInstance()

}
#endif