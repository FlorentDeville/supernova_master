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

#include "EntityTerrain.h"
#include "Graphics.h"
#include "D3D.h"
#include "Camera.h"

#include "TerrainLoader.h"
#include "TerrainData.h"

namespace Devil
{
	namespace Worlds
	{
		namespace Entities
		{
			EntityTerrain::EntityTerrain(){}

			EntityTerrain::~EntityTerrain()
			{}

			bool EntityTerrain::initialize(const string& _filename, unsigned int _tilesPerRow, unsigned int _tilesPerColumn, float quadSize,
				float _minScale, float _maxScale, IWorldEntity* _target)
			{
				//Initialize the terrain description
				if (!m_description.initFromBitmap8(_filename, _tilesPerRow, _tilesPerColumn, quadSize, _minScale, _maxScale))
					return false;

				m_target = _target;

				return true;
			}

			void EntityTerrain::update()
			{
				snVec position = m_target->getPosition();

				TileId tiles[9];

				//find the tiles we should load
				tiles[0] = m_description.getCurrentTile(position);
				unsigned int neighboorsFound = m_description.getNeighboorTiles(tiles[0], tiles + 1);

				unsigned int loadedTileCount = m_loadedTiles.size();

				//loop through the found tiles and load those which are not loaded
				for (unsigned int foundTileId = 0; foundTileId <= neighboorsFound; ++foundTileId)
				{
					//loop through the loaded tiles
					bool tileAlreadyLoaded = false;
					for (unsigned int loadedTileId = 0; loadedTileId < loadedTileCount; ++loadedTileId)
					{
						TileId tile = std::get<0>(m_loadedTiles[loadedTileId]);

						//if the tile exists then it's ok
						if (tiles[foundTileId].m_rowId == tile.m_rowId &&
							tiles[foundTileId].m_columnId == tile.m_columnId)
						{
							tileAlreadyLoaded = true;
							break;
						}

					}

					if (!tileAlreadyLoaded)
					{
						//the found tiles is not loaded so load it
						loadTile(tiles[foundTileId]);
					}
				}
			
				//loop through the loaded tiles
				vector<int> idToDelete;
				
				for (unsigned int loadedTileId = 0; loadedTileId < m_loadedTiles.size(); ++loadedTileId)
				{
					TileId tile = std::get<0>(m_loadedTiles[loadedTileId]);

					bool loadedTileObsolete = true;
					//loop through the found tiles and load those which are not loaded
					for (unsigned int foundTileId = 0; foundTileId <= neighboorsFound; ++foundTileId)
					{
						//if the tile exists then it's ok
						if (tiles[foundTileId].m_rowId == tile.m_rowId &&
							tiles[foundTileId].m_columnId == tile.m_columnId)
						{
							loadedTileObsolete = false;
							break;
						}						
					}

					if (loadedTileObsolete)
					{
						//delete the loaded tile
						unsigned int gfxId = std::get<1>(m_loadedTiles[loadedTileId]);
						GRAPHICS->releaseEntity(gfxId);
						m_loadedTiles.erase(m_loadedTiles.begin() + loadedTileId);
					}
				}
			}

			void EntityTerrain::render()
			{
				XMMATRIX proj;
				GRAPHICS->getDirectXWrapper()->getProjectionMatrix(proj);

				XMMATRIX view;
				GRAPHICS->getCamera()->GetViewMatrix(view);

				XMVECTOR color = DirectX::XMVectorSet(1, 1, 1, 1);

				XMMATRIX world = DirectX::XMMatrixIdentity();

				for (vector<TileContainer>::iterator it = m_loadedTiles.begin(); it != m_loadedTiles.end(); ++it)
				{
					unsigned int gfxId = std::get<1>(*it);
					GRAPHICS->getEntity(gfxId)->render(world, view, proj, color, 0, m_wireframe);
				}
			}

			void EntityTerrain::loadTile(const TileId& _tile)
			{
				TerrainLoader loader;
				TerrainData data;
				if (!loader.loadTile(m_description, _tile, data))
					throw;

				snVec lowerLeftCorner = m_description.computeTileLowerLeftCorner(_tile);
				unsigned int gfxId = GRAPHICS->createHeightMap(lowerLeftCorner, m_description.getQuadSize(), data.m_quadsPerRow,
					data.m_quadsPerColumn, data.m_heights);

				m_loadedTiles.push_back(TileContainer(_tile, gfxId));
			}
		}
	}
}