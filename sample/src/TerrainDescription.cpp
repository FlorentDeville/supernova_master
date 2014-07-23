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

#include "TerrainDescription.h"
#include "TileId.h"

#include <fstream>
using std::ifstream;

#include <Windows.h>

using namespace Supernova::Vector;

namespace Devil
{
	namespace Terrain
	{
		TerrainDescription::TerrainDescription() :m_filename(""), m_tilesPerRow(0), m_tilesPerColumn(0), m_vertexPerTileRow(0),
			m_vertexPerTileColumn(0), m_vertexPerColumn(0), m_vertexPerRow(0), m_quadSize(0), m_quadsPerColumn(0), m_quadsPerRow(0),
			m_quadsPerTileColumn(0), m_quadsPerTileRow(0)
		{}

		TerrainDescription::~TerrainDescription()
		{}

		string TerrainDescription::getFilename() const
		{
			return m_filename;
		}

		unsigned int TerrainDescription::getQuadsPerTileRow() const
		{
			return m_quadsPerTileRow;
		}

		unsigned int TerrainDescription::getQuadsPerTileColumn() const
		{
			return m_quadsPerTileColumn;
		}

		unsigned int TerrainDescription::getVertexPerTileRow() const
		{
			return m_vertexPerTileRow;
		}

		unsigned int TerrainDescription::getVertexPerTileColumn() const
		{
			return m_vertexPerTileColumn;
		}

		unsigned int TerrainDescription::getVertexPerRow() const
		{
			return m_vertexPerRow;
		}

		unsigned int TerrainDescription::getQuadPerRow() const
		{
			return m_quadsPerRow;
		}

		unsigned int TerrainDescription::getQuadPerColumn() const
		{
			return m_quadsPerColumn;
		}

		float TerrainDescription::getMinScale() const
		{
			return m_minScale;
		}

		float TerrainDescription::getMaxScale() const
		{
			return m_maxScale;
		}

		float TerrainDescription::getQuadSize() const
		{
			return m_quadSize;
		}

		snVec TerrainDescription::computeTileLowerLeftCorner(const TileId& _tileId) const
		{	
			snVec terrainLowerLeftCorner = snVec4Set(-(float)m_quadsPerRow * 0.5f * m_quadSize, 0, -(float)m_quadsPerColumn * 0.5f * m_quadSize, 0);

			float tileSizeRow = m_quadSize * m_quadsPerTileRow;
			float tileSizeColumn = m_quadSize * m_quadsPerTileColumn;
			return terrainLowerLeftCorner + snVec4Set(_tileId.m_columnId * tileSizeRow - m_quadSize, 0, _tileId.m_rowId * tileSizeColumn - m_quadSize, 0);
		}

		bool TerrainDescription::initFromBitmap8(const string& _filename, unsigned int _tilesPerRow, unsigned int _tilesPerColumn, 
			float _quadSize, float _minScale, float _maxScale)
		{
			m_tilesPerRow = _tilesPerRow;
			m_tilesPerColumn = _tilesPerColumn;
			m_quadSize = _quadSize;
			m_minScale = _minScale;
			m_maxScale = _maxScale;

			m_filename = _filename;

			//Load the bitmap header to get informations about the terrain size.
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

			file.close();

			m_vertexPerRow = bmpInfo->biWidth - 2;
			m_vertexPerColumn = bmpInfo->biHeight - 2;
			m_quadsPerRow = m_vertexPerRow - 1;
			m_quadsPerColumn = m_vertexPerColumn - 1;
			delete[] datBuff[0];
			delete[] datBuff[1];

			//compute the size of tiles
			m_quadsPerTileRow = m_quadsPerRow / m_tilesPerRow;
			m_quadsPerTileColumn = m_quadsPerColumn / m_tilesPerColumn;
			m_vertexPerTileRow = m_quadsPerTileRow + 1;
			m_vertexPerTileColumn = m_quadsPerTileColumn + 1;

			return true;
		}

		TileId TerrainDescription::getCurrentTile(const snVec& _position) const
		{
			float rowSize = m_quadSize * m_quadsPerRow;
			float rowTileSize = m_quadSize * m_quadsPerTileRow;

			float columnSize = m_quadSize * m_quadsPerColumn;
			float columnTileSize = m_quadSize * m_quadsPerTileColumn;

			float positionX = snVec4GetX(_position);
			float positionZ = snVec4GetZ(_position);

			TileId res;
			res.m_rowId = (unsigned int)((positionZ + (columnSize * 0.5f)) / rowTileSize);
			res.m_columnId = (unsigned int)((positionX + (rowSize * 0.5f)) / columnTileSize);

			return res;
		}

		unsigned int TerrainDescription::getNeighboorTiles(const TileId& _tileId, TileId* _neightboors) const
		{
			unsigned int neighboorsCount = 0;

			for (int columnOffset = -1; columnOffset <= 1; columnOffset++)
			{
				int currentColumn = _tileId.m_columnId + columnOffset;

				//check if the column is out of bound
				if (currentColumn < 0 || currentColumn >= m_tilesPerColumn)
					continue;

				for (int rowOffset = -1; rowOffset <= 1; rowOffset++)
				{
					int currentRow = _tileId.m_rowId + rowOffset;

					//check if the row is out of bound
					if (currentRow < 0 || currentRow >= m_tilesPerRow)
						continue;

					//same tile
					if (currentColumn == _tileId.m_columnId && currentRow == _tileId.m_rowId)
						continue;

					//neighboor found!
					_neightboors[neighboorsCount].m_rowId = currentRow;
					_neightboors[neighboorsCount].m_columnId = currentColumn;
					neighboorsCount++;
				}
			}

			return neighboorsCount;
		}
	}
}