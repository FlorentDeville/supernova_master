#include "Texture.h"

#include "DDSTextureLoader.h"

//#include <DirectXTex.h>
//#include <d3dx11tex.h>
using namespace DirectX;

namespace Devil
{
	Texture::Texture()
	{
		m_texture = 0;
	}


	Texture::~Texture()
	{
	}

	bool Texture::initialize(ID3D11Device* device, WCHAR* filename)
	{
		HRESULT result;

		// Load the texture in.
		result = CreateDDSTextureFromFile(device, filename, NULL, &m_texture);
		if (FAILED(result))
		{
			return false;
		}

		return true;
	}
	

		void Texture::shutdown()
	{
			// Release the texture resource.
			if (m_texture)
			{
				m_texture->Release();
				m_texture = 0;
			}

			return;
		}
	

		ID3D11ShaderResourceView* Texture::getTexture()
	{
			return m_texture;
		}

}