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
#ifdef _DEBUG
	#include "snLogger.h"
#endif //ifdef _DEBUG

#include "World.h"
#include "EntityTerrain.h"
#include "Graphics.h"
#include "D3D.h"
#include "Camera.h"

#include "TerrainLoader.h"
#include "TerrainData.h"

#include "TerrainCollider.h"

#include "snAABB.h"
#include "snScene.h"
#include "snRigidbody.h"
using namespace Supernova;
using namespace Supernova::Vector;

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
#ifdef _DEBUG
				bool log = false;
#endif //ifdef _DEBUG

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
#ifdef _DEBUG
						LOGGER->logInfo("Loading tile : c=" + LOGGER->toString(tiles[foundTileId].m_columnId) + " r=" + LOGGER->toString(tiles[foundTileId].m_rowId));
						log = true;
#endif //ifdef _DEBUG
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
#ifdef _DEBUG
						TileId id = std::get<0>(m_loadedTiles[loadedTileId]);
						LOGGER->logInfo("Deleting tile : c=" + LOGGER->toString(id.m_columnId) + " r=" + LOGGER->toString(id.m_rowId));
						log = true;
#endif //ifdef _DEBUG
						//delete the loaded tile
						unsigned int gfxId = std::get<1>(m_loadedTiles[loadedTileId]);
						GRAPHICS->releaseEntity(gfxId);
						snhRigidbody actor = std::get<2>(m_loadedTiles[loadedTileId]);
						WORLD->getPhysicsScene()->deleteActor(actor);
						m_loadedTiles.erase(m_loadedTiles.begin() + loadedTileId);
					}
				}

#ifdef _DEBUG
				if (log)
				{
					LOGGER->logInfo("Current tile : c=" + LOGGER->toString(tiles[0].m_columnId) + " r=" + LOGGER->toString(tiles[0].m_rowId));
					LOGGER->logInfo("============OVER============");
				}
#endif //ifdef _DEBUG
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
				//Load the tile
				TerrainLoader loader;
				TerrainData data;
				if (!loader.loadTile(m_description, _tile, data))
					throw;

				//Create the graphic entity
				snVec lowerLeftCorner = m_description.computeTileLowerLeftCorner(_tile);
				unsigned int gfxId = GRAPHICS->createHeightMap(lowerLeftCorner, m_description.getQuadSize(), data.m_quadsPerRow,
					data.m_quadsPerColumn, data.m_heights);

				//Create the physic entity
				snAABB boundingVolume;
				boundingVolume.m_min = lowerLeftCorner;
				snVec4SetY(boundingVolume.m_min, data.m_min);

				float tileSizeInUnits = m_description.getQuadSize() * data.m_quadsPerRow;
				boundingVolume.m_max = boundingVolume.m_min + snVec4Set(tileSizeInUnits, 0, tileSizeInUnits, 0);
				snVec4SetY(boundingVolume.m_max, data.m_max);

				//create the physic height map
				snhScene scene = WORLD->getPhysicsScene();

				snRigidbody* rb = new snRigidbody();
				rb->initializeStatic(Vector::VEC_ZERO, Vector::VEC_ZERO);

				snhRigidbody snMap = SUPERNOVA->registerObject(rb);
				scene->attachActor(snMap);

				TerrainCollider* collider = new	TerrainCollider(boundingVolume.m_min, boundingVolume.m_max, m_description.getQuadSize(),
					data.m_quadsPerRow, data.m_quadsPerRow, data.m_heights);
				snMap->addCollider(collider);
				snMap->getPhysicMaterial().m_friction = 1;
#ifdef _DEBUG
				snMap->setName("tile c=" + LOGGER->toString(_tile.m_columnId) + " r=" + LOGGER->toString(_tile.m_rowId));
#endif //ifdef _DEBUG
				snMap->initialize();

				//Save information about the tile
				m_loadedTiles.push_back(TileContainer(_tile, gfxId, snMap));
			}
		}
	}
}