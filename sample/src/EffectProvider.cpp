#include "EffectProvider.h"

#include "Graphics.h"
#include "D3D.h"
#include <VertexTypes.h>

namespace Devil
{
	EffectProvider* EffectProvider::m_instance = 0;

	EffectProvider::EffectProvider()
	{
	}


	EffectProvider::~EffectProvider()
	{
	}

	EffectProvider* EffectProvider::getInstance()
	{
		if (m_instance == 0)
			m_instance = new EffectProvider();

		return m_instance;
	}

	void EffectProvider::kill()
	{
		if (m_instance != 0)
		{
			delete m_instance;
			m_instance = 0;
		}
	}

	bool EffectProvider::initialize()
	{
		m_BasicEffect.reset(new BasicEffect(GRAPHICS->getDirectXWrapper()->getDevice()));
		m_BasicEffect->EnableDefaultLighting();
		m_BasicEffect->SetVertexColorEnabled(true);
		m_BasicEffect->SetTextureEnabled(false);

		void const* shaderByteCode;
		size_t byteCodeLength;
		m_BasicEffect->GetVertexShaderBytecode(&shaderByteCode, &byteCodeLength);

		ID3D11InputLayout* inputLayout = 0;

		HRESULT res = GRAPHICS->getDirectXWrapper()->getDevice()->CreateInputLayout(VertexPositionNormalColor::InputElements,
			VertexPositionNormalColor::InputElementCount, shaderByteCode, byteCodeLength, &inputLayout);
		assert(!FAILED(res));

		m_BasicEffectinputLayout.reset(inputLayout);

		return true;

	}

	BasicEffect* EffectProvider::getBasicEffect()
	{
		return m_BasicEffect.get();
	}

	ID3D11InputLayout* EffectProvider::getInputLayout()
	{
		return m_BasicEffectinputLayout.get();
	}
}