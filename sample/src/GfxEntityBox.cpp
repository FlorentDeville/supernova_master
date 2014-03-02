#include "GfxEntityBox.h"
#include "Graphics.h"
#include "D3D.h"
#include "EffectProvider.h"

#include <VertexTypes.h>
namespace Devil
{
	GfxEntityBox::GfxEntityBox()
	{
	}

	GfxEntityBox::~GfxEntityBox()
	{
	}

	bool GfxEntityBox::initialize(const XMFLOAT3& _size, const XMFLOAT4& _color)
	{
		m_size = _size;
		initializeBuffer(_color);

		m_effect = EFFECT->getBasicEffect();
		m_inputLayout = EFFECT->getInputLayout();
		return true;
	}

	void GfxEntityBox::shutdown()
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
	}

	void GfxEntityBox::render(const XMMATRIX& _world, const XMMATRIX& _view, const XMMATRIX& _projection)
	{
		unsigned int stride;
		unsigned int offset;


		// Set vertex buffer stride and offset.
		stride = sizeof(VertexPositionNormalColor);
		offset = 0;

		ID3D11DeviceContext* context = GRAPHICS->getDirectXWrapper()->getDeviceContext();

		m_effect->SetWorld(_world);
		m_effect->SetView(_view);
		m_effect->SetProjection(_projection);
		m_effect->DisableSpecular();

		m_effect->Apply(context);

		context->IASetInputLayout(m_inputLayout);

		// Set the vertex buffer to active in the input assembler so it can be rendered.
		context->IASetVertexBuffers(0, 1, &m_vertexBuffer, &stride, &offset);

		// Set the index buffer to active in the input assembler so it can be rendered.
		context->IASetIndexBuffer(m_indexBuffer, DXGI_FORMAT_R32_UINT, 0);

		// Set the type of primitive that should be rendered from this vertex buffer, in this case triangles.
		context->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

		context->DrawIndexed(m_indexCount, 0, 0);
		
		return;
	}

	bool GfxEntityBox::initializeBuffer(const XMFLOAT4& _color)
	{
		VertexPositionNormalColor* vertices;
		unsigned long* indices;
		D3D11_BUFFER_DESC vertexBufferDesc, indexBufferDesc;
		D3D11_SUBRESOURCE_DATA vertexData, indexData;
		HRESULT result;

		const int Count = 36;
		// Set the number of vertices in the vertex array.
		m_vertexCount = Count;

		// Set the number of indices in the index array.
		m_indexCount = Count;

		// Create the vertex array.
		vertices = new VertexPositionNormalColor[m_vertexCount];
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

		XMFLOAT3 halfSize(m_size.x * 0.5f, m_size.y * 0.5f, m_size.z * 0.5f);

		//front
		vertices[0].position = XMFLOAT3(halfSize.x, halfSize.y, -halfSize.z);  
		vertices[2].position = XMFLOAT3(-halfSize.x, -halfSize.y, -halfSize.z);
		vertices[1].position = XMFLOAT3(halfSize.x, -halfSize.y, -halfSize.z);
		
		vertices[3].position = XMFLOAT3(-halfSize.x, -halfSize.y, -halfSize.z);
		vertices[4].position = XMFLOAT3(-halfSize.x, halfSize.y, -halfSize.z);
		vertices[5].position = XMFLOAT3(halfSize.x, halfSize.y, -halfSize.z);

		vertices[0].normal = XMFLOAT3(0.0f, 0.0f, -1.0f);
		vertices[1].normal = XMFLOAT3(0.0f, 0.0f, -1.0f);
		vertices[2].normal = XMFLOAT3(0.0f, 0.0f, -1.0f);
		vertices[3].normal = XMFLOAT3(0.0f, 0.0f, -1.0f);
		vertices[4].normal = XMFLOAT3(0.0f, 0.0f, -1.0f);
		vertices[5].normal = XMFLOAT3(0.0f, 0.0f, -1.0f);

		vertices[0].color = _color;
		vertices[1].color = _color;
		vertices[2].color = _color;
		vertices[3].color = _color;
		vertices[4].color = _color;
		vertices[5].color = _color;

		//back
		vertices[6].position = XMFLOAT3(halfSize.x, halfSize.y, halfSize.z);
		vertices[8].position = XMFLOAT3(halfSize.x, -halfSize.y, halfSize.z);
		vertices[7].position = XMFLOAT3(-halfSize.x, -halfSize.y, halfSize.z);

		vertices[9].position = XMFLOAT3(-halfSize.x, -halfSize.y, halfSize.z);
		vertices[11].position = XMFLOAT3(-halfSize.x, halfSize.y, halfSize.z);
		vertices[10].position = XMFLOAT3(halfSize.x, halfSize.y, halfSize.z);

		vertices[6].normal = XMFLOAT3(0.0f, 0.0f, 1.0f);
		vertices[7].normal = XMFLOAT3(0.0f, 0.0f, 1.0f);
		vertices[8].normal = XMFLOAT3(0.0f, 0.0f, 1.0f);
		vertices[9].normal = XMFLOAT3(0.0f, 0.0f, 1.0f);
		vertices[10].normal = XMFLOAT3(0.0f, 0.0f, 1.0f);
		vertices[11].normal = XMFLOAT3(0.0f, 0.0f, 1.0f);

		vertices[6].color = _color;
		vertices[7].color = _color;
		vertices[8].color = _color;
		vertices[9].color = _color;
		vertices[10].color = _color;
		vertices[11].color = _color;

		//left
		vertices[12].position = XMFLOAT3(-halfSize.x, halfSize.y, halfSize.z);
		vertices[13].position = XMFLOAT3(-halfSize.x, halfSize.y, -halfSize.z);
		vertices[14].position = XMFLOAT3(-halfSize.x, -halfSize.y, -halfSize.z);

		vertices[15].position = XMFLOAT3(-halfSize.x, -halfSize.y, -halfSize.z);
		vertices[17].position = XMFLOAT3(-halfSize.x, halfSize.y, halfSize.z);
		vertices[16].position = XMFLOAT3(-halfSize.x, -halfSize.y, halfSize.z);

		vertices[12].normal = XMFLOAT3(-1.0f, 0.0f, 0.0f);
		vertices[13].normal = XMFLOAT3(-1.0f, 0.0f, 0.0f);
		vertices[14].normal = XMFLOAT3(-1.0f, 0.0f, 0.0f);
		vertices[15].normal = XMFLOAT3(-1.0f, 0.0f, 0.0f);
		vertices[16].normal = XMFLOAT3(-1.0f, 0.0f, 0.0f);
		vertices[17].normal = XMFLOAT3(-1.0f, 0.0f, 0.0f);

		vertices[12].color = _color;
		vertices[13].color = _color;
		vertices[14].color = _color;
		vertices[15].color = _color;
		vertices[16].color = _color;
		vertices[17].color = _color;

		//right
		vertices[18].position = XMFLOAT3(halfSize.x, halfSize.y, halfSize.z);
		vertices[19].position = XMFLOAT3(halfSize.x, -halfSize.y, -halfSize.z);
		vertices[20].position = XMFLOAT3(halfSize.x, halfSize.y, -halfSize.z);
		
		vertices[21].position = XMFLOAT3(halfSize.x, -halfSize.y, -halfSize.z);
		vertices[23].position = XMFLOAT3(halfSize.x, -halfSize.y, halfSize.z);
		vertices[22].position = XMFLOAT3(halfSize.x, halfSize.y, halfSize.z);
		
		vertices[18].normal = XMFLOAT3(1.0f, 0.0f, 0.0f);
		vertices[19].normal = XMFLOAT3(1.0f, 0.0f, 0.0f);
		vertices[20].normal = XMFLOAT3(1.0f, 0.0f, 0.0f);
		vertices[21].normal = XMFLOAT3(1.0f, 0.0f, 0.0f);
		vertices[22].normal = XMFLOAT3(1.0f, 0.0f, 0.0f);
		vertices[23].normal = XMFLOAT3(1.0f, 0.0f, 0.0f);

		vertices[18].color = _color;
		vertices[19].color = _color;
		vertices[20].color = _color;
		vertices[21].color = _color;
		vertices[22].color = _color;
		vertices[23].color = _color;

		//up
		vertices[24].position = XMFLOAT3(halfSize.x, halfSize.y, halfSize.z);
		vertices[25].position = XMFLOAT3(halfSize.x, halfSize.y, -halfSize.z);
		vertices[26].position = XMFLOAT3(-halfSize.x, halfSize.y, -halfSize.z);

		vertices[27].position = XMFLOAT3(-halfSize.x, halfSize.y, -halfSize.z);
		vertices[29].position = XMFLOAT3(halfSize.x, halfSize.y, halfSize.z);
		vertices[28].position = XMFLOAT3(-halfSize.x, halfSize.y, halfSize.z);

		vertices[24].normal = XMFLOAT3(0.0f, 1.0f, 0.0f);
		vertices[25].normal = XMFLOAT3(0.0f, 1.0f, 0.0f);
		vertices[26].normal = XMFLOAT3(0.0f, 1.0f, 0.0f);
		vertices[27].normal = XMFLOAT3(0.0f, 1.0f, 0.0f);
		vertices[28].normal = XMFLOAT3(0.0f, 1.0f, 0.0f);
		vertices[29].normal = XMFLOAT3(0.0f, 1.0f, 0.0f);

		vertices[24].color = _color;
		vertices[25].color = _color;
		vertices[26].color = _color;
		vertices[27].color = _color;
		vertices[28].color = _color;
		vertices[29].color = _color;

		//bottom
		vertices[30].position = XMFLOAT3(halfSize.x, -halfSize.y, halfSize.z);
		vertices[32].position = XMFLOAT3(halfSize.x, -halfSize.y, -halfSize.z);
		vertices[31].position = XMFLOAT3(-halfSize.x, -halfSize.y, -halfSize.z);

		vertices[33].position = XMFLOAT3(-halfSize.x, -halfSize.y, -halfSize.z);
		vertices[34].position = XMFLOAT3(halfSize.x, -halfSize.y, halfSize.z);
		vertices[35].position = XMFLOAT3(-halfSize.x, -halfSize.y, halfSize.z);

		vertices[30].normal = XMFLOAT3(0.0f, -1.0f, 0.0f);
		vertices[31].normal = XMFLOAT3(0.0f, -1.0f, 0.0f);
		vertices[32].normal = XMFLOAT3(0.0f, -1.0f, 0.0f);
		vertices[33].normal = XMFLOAT3(0.0f, -1.0f, 0.0f);
		vertices[34].normal = XMFLOAT3(0.0f, -1.0f, 0.0f);
		vertices[35].normal = XMFLOAT3(0.0f, -1.0f, 0.0f);

		vertices[30].color = _color;
		vertices[31].color = _color;
		vertices[32].color = _color;
		vertices[33].color = _color;
		vertices[34].color = _color;
		vertices[35].color = _color;

		for (int i = 0; i < m_indexCount; ++i)
			indices[i] = i;
		
		// Set up the description of the static vertex buffer.
		vertexBufferDesc.Usage = D3D11_USAGE_DEFAULT;
		vertexBufferDesc.ByteWidth = sizeof(VertexPositionNormalColor)* m_vertexCount;
		vertexBufferDesc.BindFlags = D3D11_BIND_VERTEX_BUFFER;
		vertexBufferDesc.CPUAccessFlags = 0;
		vertexBufferDesc.MiscFlags = 0;
		vertexBufferDesc.StructureByteStride = 0;

		// Give the subresource structure a pointer to the vertex data.
		vertexData.pSysMem = vertices;
		vertexData.SysMemPitch = 0;
		vertexData.SysMemSlicePitch = 0;

		// Now create the vertex buffer.
		result = GRAPHICS->getDirectXWrapper()->getDevice()->CreateBuffer(&vertexBufferDesc, &vertexData, &m_vertexBuffer);
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
		result = GRAPHICS->getDirectXWrapper()->getDevice()->CreateBuffer(&indexBufferDesc, &indexData, &m_indexBuffer);
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
}