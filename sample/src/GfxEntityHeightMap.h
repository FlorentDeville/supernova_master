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

#ifndef GFX_ENTITY_HEIGHTMAP_H
#define GFX_ENTITY_HEIGHTMAP_H

#include "IGfxEntity.h"

//Forward declaration of DX structures.
struct ID3D11Buffer;
struct ID3D11InputLayout;

namespace DirectX
{
	class BasicEffect;
}
using DirectX::BasicEffect;

namespace Devil
{
	//Renders a terrain using an height map as source
	class __declspec(align(16)) GfxEntityHeightMap : public IGfxEntity
	{

	private:
		//Number of indices int the index buffer
		unsigned int m_indicesCount;

		//Buffer containing the vertices making the height map
		ID3D11Buffer* m_vertexBuffer;
		
		//Buffer containing the indices forming the triangles of the height map
		ID3D11Buffer* m_indexBuffer;
		
		//Description of the inputs
		ID3D11InputLayout* m_inputLayout;

		BasicEffect* m_effect;

	public:
		//Construct the height map. It initializes the entire mesh (vertex buffer + index buffer)
		// _lowerLeftcorner : the coordinate of the lower left corner. The y axis wll be ignored to take the value from the height map.
		// _quadSize : size of a quad.
		// _width : number of quads per row (along the x axis).
		// _length : number of quads per column (along the z axis).
		// _heights : array containing the height of each vertex.
		GfxEntityHeightMap(const XMVECTOR& _lowerLeftCorner, float _quadSize, unsigned int _width, unsigned int _length, float* heights);

		virtual ~GfxEntityHeightMap();

		void shutdown();

		void render(const XMMATRIX& _world, const XMMATRIX& _view, const XMMATRIX& _projection, const XMVECTOR& _color,
			const Texture* const _texture, bool _wireframe);

		void* operator new(size_t _count);

		void operator delete(void* _p);
	};
}

#endif // ifndef GFX_ENTITY_HEIGHTMAP_H