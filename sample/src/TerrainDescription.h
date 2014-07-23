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

#ifndef TERRAIN_DESCRIPTION_H
#define TERRAIN_DESCRIPTION_H

#include <string>
using std::string;

#include "snVec.h"
using Supernova::snVec;

namespace Devil
{
	namespace Terrain
	{
		struct TileId;

		//Description of the terrain and its tiles.
		class TerrainDescription
		{
		private:
			//Name of the bitmap file
			string m_filename;

			//Number of tiles per row in the entire terrain.
			unsigned int m_tilesPerRow;

			//Number of tiles per column in the entire terrain.
			unsigned int m_tilesPerColumn;

			//Size of quad.
			float m_quadSize;

			//Number of quad in a row of a tile.
			unsigned int m_quadsPerTileRow;

			//Number of quad in a column of a tile.
			unsigned int m_quadsPerTileColumn;

			//Number of vertex in a row of a tile.
			unsigned int m_vertexPerTileRow;

			//Number of vertex in a column of a tile.
			unsigned int m_vertexPerTileColumn;

			//Number of quads per row in the entire terrain.
			unsigned int m_quadsPerRow;

			//Number of quads per column in the entire terrain.
			unsigned int m_quadsPerColumn;

			//Number of vertex per row in the entire terrain.
			unsigned int m_vertexPerRow;

			//Number of vertex per column in the entire terrain.
			unsigned int m_vertexPerColumn;

			//Minimum possible value for the height.
			float m_minScale;

			//Maximum possible value for the height.
			float m_maxScale;

		public:
			TerrainDescription();

			virtual ~TerrainDescription();

			string getFilename() const;

			unsigned int getQuadsPerTileRow() const;

			unsigned int getQuadsPerTileColumn() const;

			unsigned int getVertexPerTileRow() const;

			unsigned int getVertexPerTileColumn() const;

			unsigned int getVertexPerRow() const;

			unsigned int getQuadPerRow() const;

			unsigned int getQuadPerColumn() const;

			float getMinScale() const;

			float getMaxScale() const;

			snVec computeTileLowerLeftCorner(const TileId& _tileId) const;

			//Initialize the terrain description.
			// _filename : filename (path + filename) of the 8 bits bitmap file to use as heightmap.
			// _tilesPerRow : number of tiles in a row of the terrain.
			// _tilesPerColumn : number of tiles in a column of the terrain.
			// _quadSize : size of a single quad of the terrain.
			// _minScale : minimum value of the height.
			// _maxScale : maximum value of the height.
			// return : true if the initialization succeeded. False if an error occurred.
			bool initFromBitmap8(const string& _filename, unsigned int _tilesPerRow, unsigned int _tilesPerColumn, float _quadSize, 
				float _minScale, float _maxScale);

			//Compute and return the id of the tile overlaping the given position.
			// _position : a position in world coordinates.
			// return : the id of the tile overlapping the position.
			TileId getCurrentTile(const snVec& _position) const;

			//Compute and return the ids of the neighboor tiles of a given tile. It returns a maximum of 8 neighboors.
			// _tileId : id of a tile whose neighboor must be found.
			// _neighboors : array of minimum 8 element filled by the function with the ids of the neighboor tiles.
			// return : the number of neighboor tiles found.
			unsigned int getNeighboorTiles(const TileId& _tileId, TileId* _neightboors) const;
		};
	}
}

#endif //ifndef TERRAIN_DESCRIPTION_H