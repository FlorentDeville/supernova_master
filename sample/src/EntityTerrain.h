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

#ifndef ENTITY_TERRAIN_H
#define ENTITY_TERRAIN_H

#include "IWorldEntity.h"

#include <string>
using std::string;

#include <tuple>
using std::tuple;

#include <vector>
using std::vector;

#include "TerrainDescription.h"
#include "TileId.h"
using namespace Devil::Terrain;

namespace Devil
{
	class IGfxEntity;
	namespace Worlds
	{
		namespace Entities
		{
			typedef tuple<TileId, unsigned int> TileContainer;

			//Responsible of the terrain. It updates, streams, displays the terrain.
			class EntityTerrain : public IWorldEntity
			{
			private:
				//Description of the terrain
				TerrainDescription m_description;

				//The entity to follow.
				IWorldEntity* m_target;

				//Map between tiles ids and their graphics ids
				vector<TileContainer> m_loadedTiles;

			public:
				EntityTerrain();

				virtual ~EntityTerrain();

				//Initialize the entity. This function is mandatory before the world can use the entity.
				// _filename : path + filename of the texture to use as heightmap.
				// _tilesPerRow : number of tiles per row in the terrain.
				// _tilesPerColumn : number of tiles per column in the terrain.
				// _quadSize : size of a single quad.
				// _minScale : minimum height in the terrain.
				// _maxScale : maximum height of the terrain.
				// _target : pointer to the entity to follow.
				// return : true if the entity is ready to be used. False if an error occurred.
				bool initialize(const string& _filename, unsigned int _tilesPerRow, unsigned int _tilesPerColumn, float quadSize,
					float _minScale, float _maxScale, IWorldEntity* _target);

				void update();

				void render();
			
			private:

				//Load a tile
				// _tile : id of the tile to load.
				void loadTile(const TileId& _tile);
			};
		}
	}
}

#endif //ifndef ENTITY_TERRAIN_H