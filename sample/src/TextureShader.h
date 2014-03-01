#ifndef TEXTURE_SHADER_H
#define TEXTURE_SHADER_H

//#include <d3d11.h>
//#include <d3dx10math.h>
//#include <d3dx11async.h>
//#include <fstream>
//using namespace std;

#include <d3d11.h>
#include <DirectXMath.h>

using namespace DirectX;

namespace Devil
{
	class TextureShader
	{

	private:
		struct MatrixBufferType
		{
			XMMATRIX world;
			XMMATRIX view;
			XMMATRIX projection;
		};

		ID3D11VertexShader* m_vertexShader;
		ID3D11PixelShader* m_pixelShader;
		ID3D11InputLayout* m_layout;
		ID3D11Buffer* m_matrixBuffer;

		ID3D11SamplerState* m_sampleState;

	public:
		TextureShader();
		virtual ~TextureShader();

		bool initialize(ID3D11Device*, HWND);
		void shutdown();
		bool render(ID3D11DeviceContext*, int, XMMATRIX&, XMMATRIX&, XMMATRIX&, ID3D11ShaderResourceView*);

	private:
		bool initializeShader(ID3D11Device*, HWND, WCHAR*, WCHAR*);
		void shutdownShader();
		void outputShaderErrorMessage(ID3D10Blob*, HWND, WCHAR*);

		bool setShaderParameters(ID3D11DeviceContext*, XMMATRIX&, XMMATRIX&, XMMATRIX&, ID3D11ShaderResourceView*);
		void renderShader(ID3D11DeviceContext*, int);
	};

}

#endif