#ifndef MODEL_H
#define MODEL_H

#include <DirectXMath.h>

using DirectX::XMFLOAT3;
using DirectX::XMFLOAT2;

struct ID3D11DeviceContext;
struct ID3D11Device;
struct ID3D11ShaderResourceView;
struct ID3D11Buffer;

namespace Devil
{	
	class Texture;

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

		bool initialize(ID3D11Device*, wchar_t*);
		void shutdown();
		void render(ID3D11DeviceContext*);

		int getIndexCount();

		ID3D11ShaderResourceView* getTexture();

	private:
		bool initializeBuffers(ID3D11Device*);
		void shutdownBuffers();
		void renderBuffers(ID3D11DeviceContext*);

		bool loadTexture(ID3D11Device*, wchar_t*);
		void releaseTexture();

	};
}
#endif