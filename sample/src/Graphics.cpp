#include "Graphics.h"

#include "D3D.h"

#include "Camera.h"
#include "Model.h"
#include "ColorShader.h"
#include "Texture.h"
#include "TextureShader.h"
#include "Light.h"
#include "LightShader.h"

#include "IGfxEntity.h"
#include "GfxEntitySphere.h"
#include "GfxEntityBox.h"
#include "GfxEntityPlan.h"
#include "GfxEntityCylinder.h"
#include "GfxEntityHeightMap.h"

#include <DirectXMath.h>
#include <SpriteBatch.h>
#include <SpriteFont.h>
using namespace DirectX;

namespace Devil
{
	Graphics* Graphics::m_Instance = 0;

	Graphics* Graphics::getInstance()
	{
		if (m_Instance == 0)
			m_Instance = new Graphics();

		return m_Instance;
	}

	Graphics::Graphics() : m_screenWidth(0), m_screenHeight(0), m_clearScreenColor(Colors::Black), m_spriteBatch(0), m_spriteFontConsolas(0),
		m_idBox(255), m_idSphere(255), m_idCylinder(255)
	{
		m_D3D = 0;
		m_Camera = 0;
		m_Model = 0;
		m_ColorShader = 0;
		m_TextureShader = 0;
		m_LightShader = 0;
		m_Light = 0;
	}

	Graphics::~Graphics()
	{
		// Release the light object.
		if (m_Light)
		{
			delete m_Light;
			m_Light = 0;
		}

		// Release the light shader object.
		if (m_LightShader)
		{
			m_LightShader->shutdown();
			delete m_LightShader;
			m_LightShader = 0;
		}


		// Release the texture shader object.
		if (m_TextureShader)
		{
			m_TextureShader->shutdown();
			delete m_TextureShader;
			m_TextureShader = 0;
		}

		// Release the model object.
		if (m_Model)
		{
			m_Model->shutdown();
			delete m_Model;
			m_Model = 0;
		}

		// Release the camera object.
		if (m_Camera)
		{
			delete m_Camera;
			m_Camera = 0;
		}

		if (m_D3D)
		{
			m_D3D->shutdown();
			delete m_D3D;
			m_D3D = 0;
		}

		if (m_spriteFontConsolas != 0)
		{
			delete m_spriteFontConsolas;
			m_spriteFontConsolas = 0;
		}

		if (m_spriteBatch != 0)
		{
			delete m_spriteBatch;
			m_spriteBatch = 0;
		}

		clear();
	}

	D3D* Graphics::getDirectXWrapper()
	{
		return m_D3D;
	}

	Camera* Graphics::getCamera()
	{
		return m_Camera;
	}

	int Graphics::getScreenWidth() const
	{
		return m_screenWidth;
	}

	int Graphics::getScreenHeight() const
	{
		return m_screenHeight;
	}

	XMVECTOR Graphics::worldToScreen(const XMVECTOR& _world)
	{
		XMMATRIX viewMatrix, projectionMatrix;
		getCamera()->GetViewMatrix(viewMatrix);
		getDirectXWrapper()->getProjectionMatrix(projectionMatrix);
		return XMVector3Project(_world, 0, 0, (float)m_screenWidth, (float)m_screenHeight, 0, 1, projectionMatrix, viewMatrix, XMMatrixIdentity());
	}

	void Graphics::setClearScreenColor(const XMVECTORF32& _color)
	{
		m_clearScreenColor = _color;
	}


	void Graphics::clear()
	{
		for (std::vector<IGfxEntity*>::iterator i = m_entityList.begin(); i != m_entityList.end(); ++i)
		{
			if ((*i) != 0)
			{
				(*i)->shutdown();
				delete (*i);
				*i = 0;
			}
		}
		m_entityList.clear();

		for (std::vector<Texture*>::iterator i = m_textures.begin(); i != m_textures.end(); ++i)
		{
			if ((*i) == 0)
				continue;

			(*i)->shutdown();
			delete *i;
			*i = 0;
		}
		m_textures.clear();
	}

	IGfxEntity* Graphics::getBox()
	{
		return m_entityList[m_idBox];
	}

	IGfxEntity* Graphics::getSphere()
	{
		return m_entityList[m_idSphere];
	}

	IGfxEntity* Graphics::getCylinder()
	{
		return m_entityList[m_idCylinder];
	}

	Texture* Graphics::getTexture(unsigned int _id)
	{
		return m_textures[_id];
	}

	Texture* Graphics::getTexChecker()
	{
		return m_textures[m_IdTexChecker];
	}

	bool Graphics::initialize(int screenWidth, int screenHeight, HWND hwnd, bool _fullScreen, bool _vsync)
	{
		bool result;

		// Create the Direct3D object.
		m_D3D = new D3D();
		if (!m_D3D)
		{
			return false;
		}

		// Initialize the Direct3D object.
		result = m_D3D->initialize(screenWidth, screenHeight, _vsync, hwnd, _fullScreen, SCREEN_DEPTH, SCREEN_NEAR);
		if (!result)
		{
			MessageBox(hwnd, L"Could not initialize Direct3D", L"Error", MB_OK);
			return false;
		}

		//Create the sprite batch
		m_spriteBatch = new SpriteBatch(m_D3D->getDeviceContext());
		m_spriteFontConsolas = new SpriteFont(m_D3D->getDevice(), L"consolas.spritefont");

		m_screenWidth = screenWidth;
		m_screenHeight = screenHeight;

		// Create the camera object.
		m_Camera = new Camera();
		if (!m_Camera)
		{
			return false;
		}

		// Set the initial position of the camera.
		m_Camera->SetPosition(0.0f, 5.0f, -50.0f);

		//load meshes
		createBox();
		createSphere();
		createCylinder();

		Texture* myTexture = new Texture();
		if (!myTexture->loadWIC(L"check.png"))
		{
			MessageBox(hwnd, L"Could not load check.png", L"Error", MB_OK);
			return false;
		}
		m_IdTexChecker = m_textures.size();
		m_textures.push_back(myTexture);

		return true;
	}

	void Graphics::shutdown()
	{
		if (m_Instance != 0)
		{
			delete m_Instance;
			m_Instance = 0;
		}
	}

	void Graphics::BeginRender()
	{
		// Clear the buffers to begin the scene.
		m_D3D->beginScene(m_clearScreenColor);
	}

	void Graphics::EndRender()
	{
		// Present the rendered scene to the screen.
		m_D3D->endScene();
	}

	void Graphics::spriteBeginRender()
	{
		m_spriteBatch->Begin();
		m_D3D->TurnZBufferOff();
	}

	void Graphics::spriteEndRender()
	{
		m_spriteBatch->End();
		m_D3D->TurnZBufferOn();
	}

	void Graphics::writeText(const wstring& _text, const XMFLOAT2& _p, float _scale)
	{
		//m_spriteBatch->Begin();
		m_spriteFontConsolas->DrawString(m_spriteBatch, _text.c_str(), _p, Colors::Black, 0, XMFLOAT2(0, 0), XMFLOAT2(_scale, _scale));
		//m_spriteBatch->End();
	}

	GfxEntitySphere* Graphics::createSphere()
	{
		GfxEntitySphere* sphere = new GfxEntitySphere();
		sphere->initialize(m_D3D->getDeviceContext());
		m_idSphere = m_entityList.size();
		m_entityList.push_back(dynamic_cast<IGfxEntity*>(sphere));
		return sphere;
	}

	GfxEntityCylinder* Graphics::createCylinder()
	{
		GfxEntityCylinder* cylinder = new GfxEntityCylinder();
		cylinder->initialize(m_D3D->getDeviceContext());
		m_idCylinder = m_entityList.size();
		m_entityList.push_back(dynamic_cast<IGfxEntity*>(cylinder));
		return cylinder;
	}

	GfxEntityBox* Graphics::createBox()
	{
		GfxEntityBox* box = new GfxEntityBox();
		box->initialize(m_D3D->getDeviceContext());
		m_idBox = m_entityList.size();
		m_entityList.push_back(dynamic_cast<IGfxEntity*>(box));
		return box;
	}

	GfxEntityPlan* Graphics::createPlan(const XMFLOAT2& _size, const XMFLOAT4& _color)
	{
		GfxEntityPlan* NewPlan = new GfxEntityPlan();
		NewPlan->initialize(_size, _color);
		m_entityList.push_back(dynamic_cast<IGfxEntity*>(NewPlan));
		return NewPlan;
	}

	GfxEntityHeightMap* Graphics::createHeightMap(const XMVECTOR& _lowerLeftCorner, float _quadSize, unsigned int _width, unsigned int _length,
		float* heights)
	{
		GfxEntityHeightMap* heightMap = new GfxEntityHeightMap(_lowerLeftCorner, _quadSize, _width, _length, heights);
		m_entityList.push_back(heightMap);
		return heightMap;
	}
}