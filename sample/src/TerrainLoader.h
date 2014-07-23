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

#ifndef TERRAIN_LOADER_H
#define TERRAIN_LOADER_H


#include <string>
using std::string;

namespace Devil
{
	namespace Terrain
	{
		class TerrainData;
		class TerrainDescription;
		struct TileId;

		class TerrainLoader
		{
		public:
			TerrainLoader();

			virtual ~TerrainLoader();

			//Load a height map from a raw 16 bits bitmap file.
			// _filename : path + filename to the file.
			// _minScale : Minimum value to give to the minimum height in the map.
			// _maxScale : Maximum value to give to the maximum height in the map.
			// _data : Reference to a TerrainData object. The function will fill it with information from the file.
			// return : true if the file was loaded. False if an error occurred.
			bool loadRaw8(const string& _filename, float _minScale, float _maxScale, TerrainData& _data) const;

			//Load a height map from a 8 bits bitmap file.
			// _filename : Path + filename to the file to load.
			// _minScale : Minimum value to give to the minimum height in the map.
			// _maxScale : Maximum value to give to the maximum height in the map.
			// _data : Reference to a TerrainData object. The function will fill it with information from the file.
			// return : True if the file was loaded. False if an error occurred.
			bool loadBitmap8(const string& _filename, float _minScale, float _maxScale, TerrainData& _data) const;

			//Load a tile using information provided by a terrain description and a tile id. The loaded tile contains
			// a border of one vertex around it in order to compute the normal correctly. So the real size of the tile
			// in vertex count is the one provided in the terrain description + 2.
			// _desc : Must be a initialized TerrainDescription object.
			// _tile : The id if the tile to load.
			// _data : Reference to a TerrainData object. The function will fill it with information from the file.
			// return : True if the file was loaded. False if an error occurred.
			bool loadTile(const TerrainDescription& _desc, const TileId& _tile, TerrainData& _data) const;
		};
	}
}

#endif //ifndef TERRAIN_LOADER_H