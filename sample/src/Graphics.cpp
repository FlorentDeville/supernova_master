#include "Graphics.h"

#include "D3D.h"

#include "Camera.h"
#include "Model.h"
#include "ColorShader.h"
#include "TextureShader.h"
#include "Light.h"
#include "LightShader.h"

#include "IGfxEntity.h"
#include "GfxEntitySphere.h"
#include "GfxEntityBox.h"
#include "GfxEntityPlan.h"

#include <DirectXMath.h>
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

	void Graphics::kill()
	{
		if (m_Instance != NULL)
		{
			delete m_Instance;
			m_Instance = NULL;
		}
	}

	Graphics::Graphics() : m_screenWidth(0), m_screenHeight(0)
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

	bool Graphics::initialize(int screenWidth, int screenHeight, HWND hwnd)
	{
		bool result;

		// Create the Direct3D object.
		m_D3D = new D3D();
		if (!m_D3D)
		{
			return false;
		}

		// Initialize the Direct3D object.
		result = m_D3D->initialize(screenWidth, screenHeight, VSYNC_ENABLED, hwnd, FULL_SCREEN, SCREEN_DEPTH, SCREEN_NEAR);
		if (!result)
		{
			MessageBox(hwnd, L"Could not initialize Direct3D", L"Error", MB_OK);
			return false;
		}

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

		return true;
	}

	void Graphics::shutdown()
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

		for (std::vector<IGfxEntity*>::iterator i = m_entityList.begin(); i != m_entityList.end(); ++i)
		{
			if ((*i) != 0)
			{
				(*i)->shutdown();
				delete (*i);
			}
		}
		m_entityList.clear();
		
		return;
	}

	void Graphics::BeginRender()
	{
		// Clear the buffers to begin the scene.
		m_D3D->beginScene(0.0f, 0.0f, 0.0f, 1.0f);

		// Generate the view matrix based on the camera's position.
		//m_Camera->Render();
	}

	void Graphics::EndRender()
	{
		// Present the rendered scene to the screen.
		m_D3D->endScene();
	}

	GfxEntitySphere* Graphics::createSphere(float _diameter, const XMVECTOR& _color)
	{
		GfxEntitySphere* NewSphere = new GfxEntitySphere();
		NewSphere->initialize(m_D3D->getDeviceContext(), NULL, _diameter, _color);

		m_entityList.push_back(dynamic_cast<IGfxEntity*>(NewSphere));
		return NewSphere;
	}

	GfxEntityBox* Graphics::createBox(const XMFLOAT3& _size, const XMFLOAT4& _color)
	{
		GfxEntityBox* NewBox = new GfxEntityBox();
		NewBox->initialize(_size, _color);

		m_entityList.push_back(dynamic_cast<IGfxEntity*>(NewBox));
		return NewBox;
	}

	GfxEntityPlan* Graphics::createPlan(const XMFLOAT2& _size, const XMFLOAT4& _color)
	{
		GfxEntityPlan* NewPlan = new GfxEntityPlan();
		NewPlan->initialize(_size, _color);
		m_entityList.push_back(dynamic_cast<IGfxEntity*>(NewPlan));
		return NewPlan;
	}
}