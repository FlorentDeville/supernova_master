#ifndef GFX_ENTITY_PLAN_H
#define GFX_ENTITY_PLAN_H

#include "IGfxEntity.h"
#include <Effects.h>

using DirectX::BasicEffect;
using DirectX::XMFLOAT2;
using DirectX::XMFLOAT4;
using DirectX::XMMATRIX;

namespace Devil
{
	class GfxEntityPlan : public IGfxEntity
	{
	private:
		int m_vertexCount;
		int m_indexCount;

		ID3D11Buffer *m_vertexBuffer, *m_indexBuffer;
		ID3D11InputLayout* m_inputLayout;

		BasicEffect* m_effect;
	public:
		GfxEntityPlan();
		virtual ~GfxEntityPlan();

		bool initialize(const XMFLOAT2& _size, const XMFLOAT4& _color);
		void shutdown();
		void render(const XMMATRIX& _world, const XMMATRIX& _view, const XMMATRIX& _projection, const XMVECTOR& _color,
			const Texture* const _texture, bool _wireframe);
	};
}
#endif //GFX_ENTITY_PLAN_H