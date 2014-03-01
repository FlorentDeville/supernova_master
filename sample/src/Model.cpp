#include "Model.h"

#include <d3d11.h>
#include "Texture.h"

namespace Devil
{
	Model::Model()
	{
		m_vertexBuffer = 0;
		m_indexBuffer = 0;
		m_Texture = 0;
	}

	Model::~Model()
	{
	}

	bool Model::initialize(ID3D11Device* device, WCHAR* textureFilename)
	{
		bool result;


		// Initialize the vertex and index buffer that hold the geometry for the triangle.
		result = initializeBuffers(device);
		if (!result)
		{
			return false;
		}

		// Load the texture for this model.
		result = loadTexture(device, textureFilename);
		if (!result)
		{
			return false;
		}


		return true;
	}

	void Model::shutdown()
	{
		// Release the model texture.
		releaseTexture();

		// Release the vertex and index buffers.
		shutdownBuffers();

		return;
	}

	void Model::render(ID3D11DeviceContext* deviceContext)
	{
		// Put the vertex and index buffers on the graphics pipeline to prepare them for drawing.
		renderBuffers(deviceContext);

		return;
	}

	int Model::getIndexCount()
	{
		return m_indexCount;
	}

	ID3D11ShaderResourceView* Model::getTexture()
	{
		return m_Texture->getTexture();
	}

	bool Model::initializeBuffers(ID3D11Device* device)
	{
		VertexType* vertices;
		unsigned long* indices;
		D3D11_BUFFER_DESC vertexBufferDesc, indexBufferDesc;
		D3D11_SUBRESOURCE_DATA vertexData, indexData;
		HRESULT result;

		// Set the number of vertices in the vertex array.
		m_vertexCount = 3;

		// Set the number of indices in the index array.
		m_indexCount = 3;

		// Create the vertex array.
		vertices = new VertexType[m_vertexCount];
		if (!vertices)
		{
			return false;
		}

		// Create the index array.
		indices = new unsigned long[m_indexCount];
		if (!indices)
		{
			return false;
		}

		// Load the vertex array with data.
		vertices[0].position = XMFLOAT3(-1.0f, -1.0f, 0.0f);  // Bottom left.
		vertices[0].texture = XMFLOAT2(0.0f, 1.0f);
		vertices[0].normal = XMFLOAT3(0.0f, 0.0f, -1.0f);

		vertices[1].position = XMFLOAT3(0.0f, 1.0f, 0.0f);  // Top middle.
		vertices[1].texture = XMFLOAT2(0.5f, 0.0f);
		vertices[1].normal = XMFLOAT3(0.0f, 0.0f, -1.0f);

		vertices[2].position = XMFLOAT3(1.0f, -1.0f, 0.0f);  // Bottom right.
		vertices[2].texture = XMFLOAT2(1.0f, 1.0f);
		vertices[2].normal = XMFLOAT3(0.0f, 0.0f, -1.0f);

		// Load the index array with data.
		indices[0] = 0;  // Bottom left.
		indices[1] = 1;  // Top middle.
		indices[2] = 2;  // Bottom right.

		// Set up the description of the static vertex buffer.
		vertexBufferDesc.Usage = D3D11_USAGE_DEFAULT;
		vertexBufferDesc.ByteWidth = sizeof(VertexType)* m_vertexCount;
		vertexBufferDesc.BindFlags = D3D11_BIND_VERTEX_BUFFER;
		vertexBufferDesc.CPUAccessFlags = 0;
		vertexBufferDesc.MiscFlags = 0;
		vertexBufferDesc.StructureByteStride = 0;

		// Give the subresource structure a pointer to the vertex data.
		vertexData.pSysMem = vertices;
		vertexData.SysMemPitch = 0;
		vertexData.SysMemSlicePitch = 0;

		// Now create the vertex buffer.
		result = device->CreateBuffer(&vertexBufferDesc, &vertexData, &m_vertexBuffer);
		if (FAILED(result))
		{
			return false;
		}

		// Set up the description of the static index buffer.
		indexBufferDesc.Usage = D3D11_USAGE_DEFAULT;
		indexBufferDesc.ByteWidth = sizeof(unsigned long)* m_indexCount;
		indexBufferDesc.BindFlags = D3D11_BIND_INDEX_BUFFER;
		indexBufferDesc.CPUAccessFlags = 0;
		indexBufferDesc.MiscFlags = 0;
		indexBufferDesc.StructureByteStride = 0;

		// Give the subresource structure a pointer to the index data.
		indexData.pSysMem = indices;
		indexData.SysMemPitch = 0;
		indexData.SysMemSlicePitch = 0;

		// Create the index buffer.
		result = device->CreateBuffer(&indexBufferDesc, &indexData, &m_indexBuffer);
		if (FAILED(result))
		{
			return false;
		}

		// Release the arrays now that the vertex and index buffers have been created and loaded.
		delete[] vertices;
		vertices = 0;

		delete[] indices;
		indices = 0;

		return true;
	}

	void Model::shutdownBuffers()
	{
		// Release the index buffer.
		if (m_indexBuffer)
		{
			m_indexBuffer->Release();
			m_indexBuffer = 0;
		}

		// Release the vertex buffer.
		if (m_vertexBuffer)
		{
			m_vertexBuffer->Release();
			m_vertexBuffer = 0;
		}

		return;
	}

	void Model::renderBuffers(ID3D11DeviceContext* deviceContext)
	{
		unsigned int stride;
		unsigned int offset;


		// Set vertex buffer stride and offset.
		stride = sizeof(VertexType);
		offset = 0;

		// Set the vertex buffer to active in the input assembler so it can be rendered.
		deviceContext->IASetVertexBuffers(0, 1, &m_vertexBuffer, &stride, &offset);

		// Set the index buffer to active in the input assembler so it can be rendered.
		deviceContext->IASetIndexBuffer(m_indexBuffer, DXGI_FORMAT_R32_UINT, 0);

		// Set the type of primitive that should be rendered from this vertex buffer, in this case triangles.
		deviceContext->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

		deviceContext->DrawIndexed(m_indexCount, 0, 0);

		return;
	}

	bool Model::loadTexture(ID3D11Device* device, WCHAR* filename)
	{
		bool result;


		// Create the texture object.
		m_Texture = new Texture();
		if (!m_Texture)
		{
			return false;
		}

		// Initialize the texture object.
		result = m_Texture->initialize(device, filename);
		if (!result)
		{
			return false;
		}

		return true;
	}

	void Model::releaseTexture()
	{
		// Release the texture object.
		if (m_Texture)
		{
			m_Texture->shutdown();
			delete m_Texture;
			m_Texture = 0;
		}

		return;
	}
}