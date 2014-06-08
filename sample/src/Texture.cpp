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
#include "Texture.h"

#include "DDSTextureLoader.h"
#include "WICTextureLoader.h"
using namespace DirectX;

#include "Graphics.h"
#include "D3D.h"

namespace Devil
{
	Texture::Texture()
	{
		m_texture = 0;
	}


	Texture::~Texture()
	{
	}

	bool Texture::loadDDS(ID3D11Device* device, wchar_t* filename)
	{
		HRESULT result;

		// Load the texture in.
		result = CreateDDSTextureFromFile(device, filename, NULL, &m_texture);
		if (FAILED(result))
		{
			return false;
		}

		return true;
	}

	bool Texture::loadWIC(wchar_t* _path)
	{
		HRESULT res = CreateWICTextureFromFile(GRAPHICS->getDirectXWrapper()->getDevice(), GRAPHICS->getDirectXWrapper()->getDeviceContext(), 
			_path, NULL, &m_texture);

		if (FAILED(res))
		{
			return false;
		}

		return true;
	}
	

	void Texture::shutdown()
	{
		// Release the texture resource.
		if (m_texture)
		{
			m_texture->Release();
			m_texture = 0;
		}

		return;
	}
	

	ID3D11ShaderResourceView* Texture::getTexture() const
	{
		return m_texture;
	}

}