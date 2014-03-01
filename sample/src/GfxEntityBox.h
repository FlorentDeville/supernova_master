#ifndef GFX_ENTITY_BOX_H
#define GFX_ENTITY_BOX_H

#include "IGfxEntity.h"
#include <memory>
#include <Effects.h>

namespace Devil
{
	class GfxEntityBox : public IGfxEntity
	{
	private:
		//struct VertexType
		//{
		//	XMFLOAT3 position;
		//	XMFLOAT3 color;
		//	XMFLOAT3 normal;
		//};

		XMFLOAT3 m_size;

		int m_vertexCount;
		int m_indexCount;
		ID3D11Buffer *m_vertexBuffer, *m_indexBuffer;
		ID3D11InputLayout* m_inputLayout;

		DirectX::BasicEffect* m_effect;

	public:
		GfxEntityBox();
		virtual ~GfxEntityBox();

		bool initialize(const XMFLOAT3& _size, const XMFLOAT4& _color);
		void shutdown();
		void render(const XMMATRIX& _world, const XMMATRIX& _view, const XMMATRIX& _projection);

	private:
		bool initializeBuffer(const XMFLOAT4& _color);
	};

}
#endif