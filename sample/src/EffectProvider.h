#ifndef EFFECT_PROVIDER_H
#define EFFECT_PROVIDER_H

#include <memory>
#include <Effects.h>
#include <d3d11.h>

using DirectX::BasicEffect;

namespace Devil
{
	class EffectProvider
	{
	private:
		static EffectProvider* m_instance;

		std::unique_ptr<BasicEffect> m_BasicEffect;
		std::unique_ptr<ID3D11InputLayout> m_BasicEffectinputLayout;

	public:
		static EffectProvider* getInstance();
		static void kill();

		bool initialize();
		BasicEffect* getBasicEffect();
		ID3D11InputLayout* getInputLayout();

		virtual ~EffectProvider();

	private:
		EffectProvider();
		
	};

#define EFFECT EffectProvider::getInstance()

}
#endif //EFFECT_PROVIDER_H