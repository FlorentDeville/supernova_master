/****************************************************************************/
/*Copyright (c) 2014, Florent DEVILLE.                                      */
/*All rights reserved.                                                      */
/*                                                                          */
/*Redistribution and use in source and binary forms, with or without        */
/*modification, are permitted provided that the following conditions        */
/*are met:                                                                  */
/*                                                                          */
/* - Redistributions of source code must retain the above copyright         */
/*notice, this list of conditions and the following disclaimer.             */
/* - Redistributions in binary form must reproduce the above                */
/*copyright notice, this list of conditions and the following               */
/*disclaimer in the documentation and/or other materials provided           */
/*with the distribution.                                                    */
/* - The names of its contributors cannot be used to endorse or promote     */
/*products derived from this software without specific prior written        */
/*permission.                                                               */
/* - The source code cannot be used for commercial purposes without         */
/*its contributors' permission.                                             */
/*                                                                          */
/*THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       */
/*"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT         */
/*LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS         */
/*FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE            */
/*COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,       */
/*INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,      */
/*BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;          */
/*LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER          */
/*CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT        */
/*LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN         */
/*ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           */
/*POSSIBILITY OF SUCH DAMAGE.                                               */
/****************************************************************************/

#include "GfxEntityHeightMap.h"

#include <VertexTypes.h>
using namespace DirectX;

#include "Graphics.h"
#include "D3D.h"
#include "EffectProvider.h"

namespace Devil
{
	GfxEntityHeightMap::GfxEntityHeightMap(const XMVECTOR& _lowerLeftCorner, float _quadSize, unsigned int _width, unsigned int _length, float* heights)
	{
		//compute how many vertices we have
		m_vertexCount = (_width + 3) * (_length + 3);
		XMVECTOR* verticesPosition = (XMVECTOR*)_aligned_malloc(sizeof(XMVECTOR)* m_vertexCount, 16);// new XMVECTOR[vertexCount];
		XMVECTOR* verticesNormal = (XMVECTOR*)_aligned_malloc(sizeof(XMVECTOR)* m_vertexCount, 16);//new XMVECTOR[vertexCount];

		//fill in the array of vertices
		int vertexId = 0;	
		for (unsigned int row = 0; row <= _length+2; ++row) //loop through each row
		{
			int borderedRow = row - 1;
			XMVECTOR offsetZ = _lowerLeftCorner + (XMVectorSet(0, 0, _quadSize, 0) * (float)borderedRow);
			for (unsigned int column = 0; column <= _width+2; ++column) //loop through each column
			{
				int borderedColumn = column - 1;
				verticesPosition[vertexId] = offsetZ + (XMVectorSet(_quadSize, 0, 0, 0) * (float)(borderedColumn)) + XMVectorSet(0, heights[vertexId], 0, 0);
				verticesNormal[vertexId] = XMVectorSet(0, 0, 0, 0);
				++vertexId;
			}
		}
		
		//compute how many triangle and indices we need
		unsigned int triangleCount = (_width) * (_length) * 2;
		m_indicesCount = triangleCount * 3;

		//create the index buffer
		unsigned int indexId = 0;
		unsigned long* indices = new unsigned long[m_indicesCount];
		unsigned int vertexPerRow = _width + 3;
		for (unsigned int row = 0; row < _length + 2; ++row) //loop through each row
		{
			unsigned int offsetId = row * vertexPerRow;
			for (unsigned int column = 0; column < _width + 2; ++column)//loop through each column
			{
				unsigned int id0 = offsetId + column;
				unsigned int id1 = offsetId + column + vertexPerRow + 1;
				unsigned int id2 = offsetId + column + 1;
				if (row != 0 && row != _length + 1 && column != 0 && column != _width + 1)
				{
					//lower right triangle
					indices[indexId] = id0;
					indices[indexId + 1] = id1;
					indices[indexId + 2] = id2;
					indexId += 3;
				}
				//compute normals
				XMVECTOR v0 = verticesPosition[id0];
				XMVECTOR v1 = verticesPosition[id1];
				XMVECTOR v2 = verticesPosition[id2];

				XMVECTOR normal = XMVector3Cross(v0 - v1, v2 - v1);
				normal = XMVector3Normalize(normal);
				
				verticesNormal[id0] += normal;
				verticesNormal[id1] += normal;
				verticesNormal[id2] += normal;
				
			
				id0 = offsetId + column;
				id1 = offsetId + column + vertexPerRow;
				id2 = offsetId + column + vertexPerRow + 1;
				if (row != 0 && row != _length + 1 && column != 0 && column != _width + 1)
				{
					indices[indexId] = id0;
					indices[indexId + 1] = id1;
					indices[indexId + 2] = id2;
					indexId += 3;
				}
				//compute normals
				v0 = verticesPosition[id0];
				v1 = verticesPosition[id1];
				v2 = verticesPosition[id2];

				normal = XMVector3Cross(v0 - v1, v2 - v1);
				normal = XMVector3Normalize(normal);

				verticesNormal[id0] += normal;
				verticesNormal[id1] += normal;
				verticesNormal[id2] += normal;			
			}
		}
		
		//compute normals
		//unsigned int vertexPerRow = _length + 3;
		//unsigned int vertexPerColumn = _length + 3;

		for (unsigned int row = 0; row <= _length+2; ++row) //loop through each row
		{
			unsigned int offset = row * (_width + 3);
			for (unsigned int column = 0; column <= _width+2; ++column)//loop through each column
			{
				unsigned int vertexId = offset + column;

				//case where a vertex is shared by 1 triangle
				if ((row == 0 && column == _width+2) ||			//bottom right corner
					(row == _length+2 && column == 0))			//top left corner
				{
					//nothing to do
				}
				//case where a vertex is shared by 2 triangles
				else if ((row == 0 && column == 0) ||			//bottom left corner
					(row == _length+2 && column == _width+2))		//top right corner
				{
					verticesNormal[vertexId] = verticesNormal[vertexId] / 2;
				}
				//case where a vertex is shared by 3 triangles
				else if (row == 0 || row == _length+2 || column == 0 || column == _width+2)	//border
				{
					verticesNormal[vertexId] = verticesNormal[vertexId] / 3;
				}
				else //the inside of the height map. Each vertex is shared by 6 triangles
				{
					verticesNormal[vertexId] = verticesNormal[vertexId] / 6;
				}

				verticesNormal[vertexId] = XMVector3Normalize(verticesNormal[vertexId]);
			}

		}

		//fill in dx structures
		VertexPositionNormalColor* vertices = new VertexPositionNormalColor[m_vertexCount];
		for (unsigned int i = 0; i < m_vertexCount; ++i)
		{
			XMStoreFloat3(&vertices[i].position, verticesPosition[i]);
			XMStoreFloat3(&vertices[i].normal, verticesNormal[i]);

			XMFLOAT4 myColor;
			myColor.x = 0.4f;
			myColor.y = 0.4f;
			myColor.z = abs((vertices[i].position.y + 200) / 1000.f)+0.1f;
			myColor.w = 1.f;
			vertices[i].color = myColor;
		}


		// Set up the description of the static vertex buffer.
		D3D11_BUFFER_DESC vertexBufferDesc;
		vertexBufferDesc.Usage = D3D11_USAGE_DEFAULT;
		vertexBufferDesc.ByteWidth = sizeof(VertexPositionNormalColor)* m_vertexCount;
		vertexBufferDesc.BindFlags = D3D11_BIND_VERTEX_BUFFER;
		vertexBufferDesc.CPUAccessFlags = 0;
		vertexBufferDesc.MiscFlags = 0;
		vertexBufferDesc.StructureByteStride = 0;

		// Give the subresource structure a pointer to the vertex data.
		D3D11_SUBRESOURCE_DATA vertexData;
		vertexData.pSysMem = vertices;
		vertexData.SysMemPitch = 0;
		vertexData.SysMemSlicePitch = 0;

		// Now create the vertex buffer.
		HRESULT result = GRAPHICS->getDirectXWrapper()->getDevice()->CreateBuffer(&vertexBufferDesc, &vertexData, &m_vertexBuffer);
		if (FAILED(result))
			throw;

		// Set up the description of the static index buffer.
		D3D11_BUFFER_DESC indexBufferDesc;
		indexBufferDesc.Usage = D3D11_USAGE_DEFAULT;
		indexBufferDesc.ByteWidth = sizeof(unsigned long)* m_indicesCount;
		indexBufferDesc.BindFlags = D3D11_BIND_INDEX_BUFFER;
		indexBufferDesc.CPUAccessFlags = 0;
		indexBufferDesc.MiscFlags = 0;
		indexBufferDesc.StructureByteStride = 0;

		// Give the subresource structure a pointer to the index data.
		D3D11_SUBRESOURCE_DATA indexData;
		indexData.pSysMem = indices;
		indexData.SysMemPitch = 0;
		indexData.SysMemSlicePitch = 0;
		
		// Create the index buffer.
		result = GRAPHICS->getDirectXWrapper()->getDevice()->CreateBuffer(&indexBufferDesc, &indexData, &m_indexBuffer);
		if (FAILED(result))
			throw;

		// Release the arrays now that the vertex and index buffers have been created and loaded.
		_aligned_free(verticesPosition);
		_aligned_free(verticesNormal);
		delete[] vertices;
		delete[] indices;

		m_effect = EFFECT->getBasicEffect();
		m_inputLayout = EFFECT->getInputLayout();

	}

	GfxEntityHeightMap::~GfxEntityHeightMap()
	{}

	void GfxEntityHeightMap::shutdown()
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

	void GfxEntityHeightMap::render(const XMMATRIX& _world, const XMMATRIX& _view, const XMMATRIX& _projection, const XMVECTOR& _color,
		const Texture* const _texture, bool _wireframe)
	{
		UNREFERENCED_PARAMETER(_color);
		UNREFERENCED_PARAMETER(_texture);

		// Set vertex buffer stride and offset.
		unsigned int stride = sizeof(VertexPositionNormalColor);
		unsigned int offset = 0;

		if (_wireframe)
			GRAPHICS->getDirectXWrapper()->turnOnWireframeMode();


		ID3D11DeviceContext* context = GRAPHICS->getDirectXWrapper()->getDeviceContext();
		m_effect->SetWorld(_world);
		m_effect->SetView(_view);
		m_effect->SetProjection(_projection);
		m_effect->DisableSpecular();
		m_effect->SetAmbientLightColor(XMVectorSet(1, 1, 1, 1));
		m_effect->Apply(context);
		
		context->IASetInputLayout(m_inputLayout);

		// Set the vertex buffer to active in the input assembler so it can be rendered.
		context->IASetVertexBuffers(0, 1, &m_vertexBuffer, &stride, &offset);
		
		// Set the index buffer to active in the input assembler so it can be rendered.
		context->IASetIndexBuffer(m_indexBuffer, DXGI_FORMAT_R32_UINT, 0);
		
		// Set the type of primitive that should be rendered from this vertex buffer, in this case triangles.
		context->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
		context->DrawIndexed(m_indicesCount, 0, 0);

		if (_wireframe)
			GRAPHICS->getDirectXWrapper()->turnOnFillMode();
	}

	unsigned int GfxEntityHeightMap::getVertexCount() const
	{
		return m_vertexCount;
	}

	unsigned int GfxEntityHeightMap::getIndicesCount() const
	{
		return m_indicesCount;
	}

	void* GfxEntityHeightMap::operator new(size_t _count)
	{
		return _aligned_malloc(_count, 16);
	}

	void GfxEntityHeightMap::operator delete(void* _p)
	{
		_aligned_free(_p);
	}
}