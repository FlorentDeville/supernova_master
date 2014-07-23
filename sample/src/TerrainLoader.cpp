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

#include "TerrainLoader.h"
#include "TerrainData.h"
#include "TerrainDescription.h"
#include "TileId.h"

#include <fstream>
using std::ifstream;

#include "snMath.h"

#include <Windows.h>

namespace Devil
{
	namespace Terrain
	{
		TerrainLoader::TerrainLoader()
		{}

		TerrainLoader::~TerrainLoader()
		{}

		bool TerrainLoader::loadRaw8(const string& _filename, float _minScale, float _maxScale, TerrainData& _data) const
		{
			_data.m_min = SN_FLOAT_MAX;
			_data.m_max = -SN_FLOAT_MAX;

			//open the file and put the stream to the end
			std::ifstream file;
			file.open(_filename.c_str(), std::ios::in | std::ios::binary | std::ios::ate);
			if (!file.is_open())
				return false;

			//get the size of the file
			unsigned int fileSize = (unsigned int)file.tellg();

			//compute the number of vertex
			unsigned int vertexCount = fileSize; //each vertex is on 8 bits, 1 bytes

			//compute the number of vertices per row and column
			unsigned int size = (unsigned int)sqrtf((float)vertexCount);
			_data.m_vertexPerColumn = size;
			_data.m_vertexPerRow = size;

			_data.m_quadsPerRow = _data.m_vertexPerRow - 1;
			_data.m_quadsPerColumn = _data.m_vertexPerColumn - 1;

			//create the array
			_data.m_heights = new float[vertexCount];

			//Read the entire file and close it
			file.seekg(0, std::ios::beg);
			unsigned char* buffer = new unsigned char[vertexCount];
			file.read((char*)buffer, vertexCount);
			file.close();

			//Compute the scale
			const float UNSIGNED_CHAR_MAX_VALUE = 255.f;
			float deltaScale = (_maxScale - _minScale) / UNSIGNED_CHAR_MAX_VALUE;

			//Loop through each vertex
			for (unsigned int i = 0; i < vertexCount; ++i)
			{
				_data.m_heights[i] = (deltaScale * (float)buffer[i]) + _minScale;
				if (_data.m_heights[i] > _data.m_max) _data.m_max = _data.m_heights[i];
				if (_data.m_heights[i] < _data.m_min) _data.m_min = _data.m_heights[i];
			}

			return true;
		}

		bool TerrainLoader::loadBitmap8(const string& _filename, float _minScale, float _maxScale, TerrainData& _data) const
		{
			//Open the file
			std::ifstream file;
			file.open(_filename, std::ios::binary);
			if (!file.is_open())
			{
				return false;
			}

			//Allocate byte memory that will hold the two headers
			unsigned char* datBuff[2] = { 0, 0 };
			datBuff[0] = new unsigned char[sizeof(BITMAPFILEHEADER)];
			datBuff[1] = new unsigned char[sizeof(BITMAPINFOHEADER)];

			file.read((char*)datBuff[0], sizeof(BITMAPFILEHEADER));
			file.read((char*)datBuff[1], sizeof(BITMAPINFOHEADER));

			//Construct the values from the buffers
			BITMAPFILEHEADER* bmpHeader = (BITMAPFILEHEADER*)datBuff[0];
			BITMAPINFOHEADER* bmpInfo = (BITMAPINFOHEADER*)datBuff[1];
			//Check if the file is an actual BMP file
			if (bmpHeader->bfType != 0x4D42)
			{
				delete[] datBuff[0];
				delete[] datBuff[1];
				return false;
			}

			//First allocate pixel memory
			unsigned int vertexCount = bmpInfo->biWidth * bmpInfo->biHeight;
			unsigned int pixelsBufferSize = bmpHeader->bfSize - bmpHeader->bfOffBits;
			unsigned char* pixels = new unsigned char[pixelsBufferSize];

			//Go to where image data starts, then read in image data
			file.seekg(bmpHeader->bfOffBits, std::ios::beg);
			file.read((char*)pixels, pixelsBufferSize);

			file.close();

			//Initialise the TerrainData structure
			_data.m_min = SN_FLOAT_MAX;
			_data.m_max = -SN_FLOAT_MAX;

			_data.m_vertexPerRow = bmpInfo->biWidth;
			_data.m_vertexPerColumn = bmpInfo->biHeight;
			_data.m_quadsPerRow = _data.m_vertexPerRow - 1;
			_data.m_quadsPerColumn = _data.m_vertexPerColumn - 1;

			//Compute the scale
			const float UNSIGNED_CHAR_MAX_VALUE = 255.f;
			float deltaScale = (_maxScale - _minScale) / UNSIGNED_CHAR_MAX_VALUE;

			//Compute the padding added at the end of each scanline
			unsigned int padding = 4 - (_data.m_vertexPerRow % 4);

			//Create the height buffer
			_data.m_heights = new float[vertexCount];
			unsigned int vertexId = 0;
			unsigned int pixelId = 0;
			for (unsigned int row = 0; row < _data.m_vertexPerRow; ++row) //Loop through each line
			{
				pixelId = row * (_data.m_vertexPerRow + padding); //Compute the offset with padding
				for (unsigned int column = 0; column < _data.m_vertexPerColumn; ++column) //Loop through each column.
				{
					_data.m_heights[vertexId] = (deltaScale * (float)pixels[pixelId]) + _minScale;
					if (_data.m_heights[vertexId] < _data.m_min) _data.m_min = _data.m_heights[vertexId];
					if (_data.m_heights[vertexId] > _data.m_max) _data.m_max = _data.m_heights[vertexId];

					++vertexId;
					++pixelId;
				}
			}

			delete[] datBuff[0];
			delete[] datBuff[1];
			delete[] pixels;

			return true;
		}

		bool TerrainLoader::loadTile(const TerrainDescription& _desc, const TileId& _tile, TerrainData& _data) const
		{
			//Open the file
			std::ifstream file;
			file.open(_desc.getFilename(), std::ios::binary);
			if (!file.is_open())
			{
				return false;
			}

			//Load the header
			unsigned char* datBuff = 0;
			datBuff = new unsigned char[sizeof(BITMAPFILEHEADER)];
			file.read((char*)datBuff, sizeof(BITMAPFILEHEADER));
			BITMAPFILEHEADER* bmpHeader = (BITMAPFILEHEADER*)datBuff;

			//Allocate enough memory to load a scanline
			unsigned char* pixels = new unsigned char[_desc.getVertexPerTileRow() + 2];

			//Allocate memory to load the height map
			_data.m_heights = new float[(_desc.getVertexPerTileColumn() + 2) * (_desc.getVertexPerTileRow() + 2)];

			//Number of vertex to consider as the border
			const int BORDER = 1;

			unsigned int tileSizeInQuads = _desc.getQuadsPerTileRow();
			unsigned int realVertexCountPerLine = _desc.getVertexPerRow() + BORDER * 2;

			//Compute the padding added at the end of each scanline (could be cached)
			unsigned int padding = 4 - (realVertexCountPerLine % 4);

			//The size in bytes of an entire line of the texture including the padding (could be cached)
			unsigned int terrainScanlineSize = realVertexCountPerLine + padding;

			//Offset in bytes from the start of line to the first vertex of the tile
			unsigned int offsetFromStartOfLine = _tile.m_columnId * tileSizeInQuads;

			//Offset in bytes from the start of the file to the first vertex of the tile
			unsigned int offsetFromStartOfFile = _tile.m_rowId * tileSizeInQuads * terrainScanlineSize + offsetFromStartOfLine;
			
			//Offset in bytes from the last vertex of a line to the first vertex of the next line. (could be cached)
			unsigned int offsetNextScanLine = terrainScanlineSize - (_desc.getVertexPerTileRow() + 2);
			//_desc.getVertexPerRow() - _desc.getVertexPerTileRow() + padding + BORDER * 2;

			//Compute the scale (could be cached)
			const float UNSIGNED_CHAR_MAX_VALUE = 255.f;
			float deltaScale = (_desc.getMaxScale() - _desc.getMinScale()) / UNSIGNED_CHAR_MAX_VALUE;

			//Go to where image data starts, then read in image data
			file.seekg(bmpHeader->bfOffBits + offsetFromStartOfFile, std::ios::beg);

			//Loop through each scanline
			unsigned int heightId = 0;
			for (unsigned int scanline = 0; scanline < _desc.getVertexPerTileColumn() + 2; ++scanline)
			{
				file.read((char*)pixels, _desc.getVertexPerTileRow() + 2);

				//Loop through each vertex
				for (unsigned int pixelId = 0; pixelId < _desc.getVertexPerTileRow() + 2; pixelId++)
				{
					_data.m_heights[heightId] = (deltaScale * (float)pixels[pixelId]) + _desc.getMinScale();
					if (_data.m_heights[heightId] < _data.m_min) _data.m_min = _data.m_heights[heightId];
					if (_data.m_heights[heightId] > _data.m_max) _data.m_max = _data.m_heights[heightId];

					++heightId;
				}

				file.seekg(offsetNextScanLine, std::ios::cur);
			}
		
			file.close();

			_data.m_quadsPerColumn = _desc.getQuadsPerTileColumn();
			_data.m_quadsPerRow = _desc.getQuadsPerTileRow();
			_data.m_vertexPerColumn = _desc.getVertexPerTileColumn();
			_data.m_vertexPerRow = _desc.getVertexPerTileRow();

			delete[] pixels;
			delete datBuff;
			return true;
		}
	}
}
