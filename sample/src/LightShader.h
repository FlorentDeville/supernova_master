#ifndef LIGHT_SHADER_H
#define LIGHT_SHADER_H

#include <DirectXMath.h>
#include <d3d11.h>

using DirectX::XMMATRIX;
using DirectX::XMFLOAT4;
using DirectX::XMFLOAT3;

namespace Devil
{
	class LightShader
	{
	private:
		struct MatrixBufferType
		{
			XMMATRIX world;
			XMMATRIX view;
			XMMATRIX projection;
		};

		struct LightBufferType
		{
			XMFLOAT4 diffuseColor;
			XMFLOAT3 lightDirection;
			float padding;  // Added extra padding so structure is a multiple of 16 for CreateBuffer function requirements.
		};

		ID3D11VertexShader* m_vertexShader;
		ID3D11PixelShader* m_pixelShader;
		ID3D11InputLayout* m_layout;
		ID3D11SamplerState* m_sampleState;
		ID3D11Buffer* m_matrixBuffer;
		ID3D11Buffer* m_lightBuffer;

	public:
		LightShader();
		LightShader(const LightShader&);
		~LightShader();

		bool initialize(ID3D11Device*, HWND);
		void shutdown();
		bool render(ID3D11DeviceContext*, int, XMMATRIX&, XMMATRIX&, XMMATRIX&, ID3D11ShaderResourceView*, XMFLOAT3&, XMFLOAT4&);

	private:
		bool initializeShader(ID3D11Device*, HWND, WCHAR*, WCHAR*);
		void shutdownShader();
		void outputShaderErrorMessage(ID3D10Blob*, HWND, WCHAR*);

		bool setShaderParameters(ID3D11DeviceContext*, XMMATRIX&, XMMATRIX&, XMMATRIX&, ID3D11ShaderResourceView*, XMFLOAT3&, XMFLOAT4&);
		void renderShader(ID3D11DeviceContext*, int);

	
	};

}
#endif //LIGHT_SHADER_H