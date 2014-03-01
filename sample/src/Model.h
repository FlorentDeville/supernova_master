#ifndef MODEL_H
#define MODEL_H

#include <DirectXMath.h>
#include "Texture.h"

using namespace DirectX;

//class ID3D11Device;
//class ID3D11DeviceContext;
//class ID3D11Buffer;
//class Texture;

namespace Devil
{	
	class Model
	{
	private:

		struct VertexType
		{
			XMFLOAT3 position;
			XMFLOAT2 texture;
			XMFLOAT3 normal;
		};

		ID3D11Buffer *m_vertexBuffer, *m_indexBuffer;
		int m_vertexCount, m_indexCount;
		Texture* m_Texture;

	public:
		Model();
		~Model();

		bool initialize(ID3D11Device*, WCHAR*);
		void shutdown();
		void render(ID3D11DeviceContext*);

		int getIndexCount();

		ID3D11ShaderResourceView* getTexture();

	private:
		bool initializeBuffers(ID3D11Device*);
		void shutdownBuffers();
		void renderBuffers(ID3D11DeviceContext*);

		bool loadTexture(ID3D11Device*, WCHAR*);
		void releaseTexture();

	};
}
#endif