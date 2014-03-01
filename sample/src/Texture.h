#ifndef TEXTURE_H
#define TEXTURE_H

#include <d3d11.h>

namespace Devil
{
	class Texture
	{
	private:
		ID3D11ShaderResourceView* m_texture;

	public:
		Texture();
		virtual ~Texture();

		bool initialize(ID3D11Device*, WCHAR*);
		void shutdown();
		
		ID3D11ShaderResourceView* getTexture();

	
	};

}

#endif