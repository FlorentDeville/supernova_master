#include "GfxEntityPlan.h"

#include <VertexTypes.h>

#include "Graphics.h"
#include "D3D.h"

#include "EffectProvider.h"
#include <Effects.h>

namespace Devil
{
	GfxEntityPlan::GfxEntityPlan()
	{
	}


	GfxEntityPlan::~GfxEntityPlan()
	{
	}

	bool GfxEntityPlan::initialize(const XMFLOAT2& _size, const XMFLOAT4& _color)
	{
		VertexPositionNormalColor* vertices;
		unsigned long* indices;
		D3D11_BUFFER_DESC vertexBufferDesc, indexBufferDesc;
		D3D11_SUBRESOURCE_DATA vertexData, indexData;
		HRESULT result;

		m_vertexCount = 4;
		m_indexCount = 6;

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

		float halfWidth = _size.x * 0.5f;
		float halfHeight = _size.y * 0.5f;
		
		vertices[0].position = XMFLOAT3(-halfWidth, 0, halfHeight);
		vertices[1].position = XMFLOAT3(-halfWidth, 0, -halfHeight);
		vertices[2].position = XMFLOAT3(halfWidth, 0, -halfHeight);
		vertices[3].position = XMFLOAT3(halfWidth, 0, halfHeight);

		vertices[0].normal = XMFLOAT3(0.0f, 1.0f, 0.0f);
		vertices[1].normal = XMFLOAT3(0.0f, 1.0f, 0.0f);
		vertices[2].normal = XMFLOAT3(0.0f, 1.0f, 0.0f);
		vertices[3].normal = XMFLOAT3(0.0f, 1.0f, 0.0f);
		
		vertices[0].color = _color;
		vertices[1].color = _color;
		vertices[2].color = _color;
		vertices[3].color = _color;
		
		indices[0] = 0;
		indices[1] = 2;
		indices[2] = 1;

		indices[3] = 2;
		indices[4] = 0;
		indices[5] = 3;

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

		m_effect = EFFECT->getBasicEffect();
		m_inputLayout = EFFECT->getInputLayout();
		return true;
	}

	void GfxEntityPlan::shutdown()
	{

	}

	void GfxEntityPlan::render(const XMMATRIX& _world, const XMMATRIX& _view, const XMMATRIX& _projection)
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
}