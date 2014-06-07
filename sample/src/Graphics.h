#ifndef GRAPHICS_H
#define GRAPHICS_H

#define WIN32_LEAN_AND_MEAN
#include <Windows.h>

#include <vector>
#include "IGfxEntity.h"
#include <DirectXColors.h>

#include <string>
using std::wstring;

namespace DirectX
{
	class SpriteBatch;
	class SpriteFont;
}

using DirectX::XMVECTORF32;
using DirectX::SpriteBatch;
using DirectX::SpriteFont;
using DirectX::XMFLOAT2;
using DirectX::XMFLOAT3;
using DirectX::XMFLOAT4;
using DirectX::XMVECTOR;

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

	const float SCREEN_DEPTH = 10000.0f;
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

		//DirectX object used to writ text on screen
		SpriteBatch* m_spriteBatch;

		//font consolas used to write text on screen
		SpriteFont* m_spriteFontConsolas;
	public:
		static Graphics* getInstance();

		
		~Graphics();

		bool initialize(int _screenWidth, int _screenHeight, HWND _hwnd, bool _fullScreen, bool _vsync);
		void shutdown();
		
		void BeginRender();
		void EndRender();

		//Start rendering of sprites
		void spriteBeginRender();

		//end rendering of sprites.
		void spriteEndRender();

		// <summary>
		// Write text on screen
		// </summary>
		// <param name="_text">the text to write on screen</param>
		// <param name="_p">Position of the text on screen.</param>
		// <param name="_scale">Uniform scaling to apply to the text.</param>
		void writeText(const wstring& _text, const XMFLOAT2& _p, float _scale);

		GfxEntitySphere* createSphere(float _diameter, const XMVECTOR& _color = DirectX::Colors::White);
		GfxEntityBox* createBox(const XMFLOAT3&, const XMFLOAT4&);
		GfxEntityPlan* createPlan(const XMFLOAT2&, const XMFLOAT4&);

		D3D* getDirectXWrapper();
		Camera* getCamera();

		//Get the width of the screen in pixel.
		int getScreenWidth() const;

		//Get the height of the screen in pixel.
		int getScreenHeight() const;

		//Convert a world space position into a screen space position
		XMVECTOR worldToScreen(const XMVECTOR& _world);

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