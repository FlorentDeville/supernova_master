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

#include <fstream>
using std::ifstream;

#include "snMath.h"

namespace Devil
{
	namespace Terrain
	{
		TerrainLoader::TerrainLoader()
		{}

		TerrainLoader::~TerrainLoader()
		{}

		bool TerrainLoader::loadRaw8(const string& _filename, TerrainData& _data)
		{
			const unsigned int DEPTH = 1;

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

			//loop through the file
			file.seekg(0, std::ios::beg);

			unsigned int vertexId = 0;
			while (vertexId < vertexCount)
			{
				short buffer = 0;
				file.read((char*)&buffer, DEPTH);
				float f = buffer;
				if (f > _data.m_max) _data.m_max = f;
				if (f < _data.m_min) _data.m_min = f;

				_data.m_heights[vertexId++] = f;
			}

			file.close();

			//scale the heights
			float deltaData = _data.m_max - _data.m_min;
			float deltaHeight = 1200;
			float ratio = deltaHeight / deltaData;

			for (unsigned int i = 0; i < vertexCount; ++i)
				_data.m_heights[i] *= ratio;

			_data.m_max *= ratio;
			_data.m_min *= ratio;

			return true;
		}
	}
}
