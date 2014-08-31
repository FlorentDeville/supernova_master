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

#include "snHeightMap.h"
using namespace Supernova::Vector;

#include "snMatrix44f.h"
#include "snTransform.h"

namespace Supernova
{
	snHeightMap::snHeightMap(const snVec& _min, const snVec& _max, float _quadSize, unsigned int _width, unsigned int _length)
	{
		m_typeOfCollider = snEColliderHeightMap;
		m_boundingVolume.m_min = _min;
		m_boundingVolume.m_max = _max;
		m_quadSize = _quadSize;
		m_width = _width;
		m_length = _length;
	}

	snHeightMap::~snHeightMap(){}

	void snHeightMap::initialize(){}

	void snHeightMap::updateFromTransform()
	{
	}

	void snHeightMap::computeLocalInertiaTensor(float _mass, snMatrix44f& _inertiaTensor) const
	{
		SN_UNREFERENCED_PARAMETER(_mass);
		SN_UNREFERENCED_PARAMETER(_inertiaTensor);
	}

	void snHeightMap::computeAABB(snAABB * const _boundingVolume) const
	{
		_boundingVolume->m_min = m_boundingVolume.m_min;
		_boundingVolume->m_max = m_boundingVolume.m_max;
	}

	int snHeightMap::getOverlapTriangles(const snAABB& _bounding, unsigned int* const _ids, unsigned int _maxTriangles) const
	{
		//compute the indices of the quads
		snVec originMin = _bounding.m_min - m_boundingVolume.m_min;
		snVec originMax = _bounding.m_max - m_boundingVolume.m_min;

		unsigned int startColumn = unsigned int(snVec4GetX(originMin) / m_quadSize);
		unsigned int startRow = unsigned int(snVec4GetZ(originMin) / m_quadSize);

		if (startRow < 0)
			startRow = 0;
		if (startColumn < 0)
			startColumn = 0;

		unsigned int endColumn = unsigned int(snVec4GetX(originMax) / m_quadSize);
		unsigned int endRow = unsigned int(snVec4GetZ(originMax) / m_quadSize);
		
		if (endRow >= m_length)
			endRow = m_length - 1;
		if (endColumn >= m_width)
			endColumn = m_width - 1;

		//from the indices of the quads, compute the indices of the triangles
		unsigned int currentIndex = 0;
		unsigned int twoWidth = 2 * m_width;
		for (unsigned int row = startRow; row <= endRow; ++row)//loop through each row
		{
			unsigned int offsetY = row * twoWidth;
			for (unsigned int column = startColumn; column <= endColumn; ++column) //loop through each column
			{
				unsigned int newId = offsetY + (column * 2);
				if (currentIndex < _maxTriangles)
				{
					_ids[currentIndex++] = newId;
				}
				else
					break;

				++newId;
				if (currentIndex < _maxTriangles)
				{
					_ids[currentIndex++] = newId;
				}
				else
					break;

			}
		}

		return currentIndex;

	}

	int snHeightMap::getOverlapTriangles(const snVec& _point, unsigned int* const _ids, unsigned int _maxTriangles) const
	{
		//compute the indices of the quads
		snVec offsetedPoint = _point - m_boundingVolume.m_min;

		unsigned int quadColumn = unsigned int(snVec4GetX(offsetedPoint) / m_quadSize);
		unsigned int quadRow = unsigned int(snVec4GetZ(offsetedPoint) / m_quadSize);

		if (quadRow < 0 || quadRow >= m_length)
			return 0;
		if (quadColumn < 0 || quadColumn >= m_width)
			return 0;

		//from the indices of the quad, compute the indices of the triangles
		unsigned int twoWidth = 2 * m_width;
		unsigned int offsetY = quadRow * twoWidth;
		unsigned int newId = offsetY + (quadColumn * 2);

		unsigned int currentIndex = 0;
		if (currentIndex < _maxTriangles)
		{
			_ids[currentIndex++] = newId;
		}
		else
		{
			return currentIndex;
		}

		++newId;
		if (currentIndex < _maxTriangles)
		{
			_ids[currentIndex++] = newId;
		}

		return currentIndex;
	}

	snAABB const & snHeightMap::getBoundingVolume() const
	{
		return m_boundingVolume;
	}

	float snHeightMap::getQuadSize() const
	{
		return m_quadSize;
	}

	unsigned int snHeightMap::getLength() const
	{
		return m_length;
	}

	unsigned int snHeightMap::getWidth() const
	{
		return m_width;
	}

	bool snHeightMap::getOverlapQuad(const snVec& _point, unsigned int& _x, unsigned int& _y) const
	{
		//compute the indices of the quads
		snVec offsetedPoint = _point - m_boundingVolume.m_min;

		_x = unsigned int(snVec4GetX(offsetedPoint) / m_quadSize);
		_y = unsigned int(snVec4GetZ(offsetedPoint) / m_quadSize);

		if (_y < 0 || _y >= m_length || _x < 0 || _x >= m_width)
			return false;
		
		return true;
	}

	void snHeightMap::getTriangleIds(unsigned int _x, unsigned int _z, unsigned int* const _ids) const
	{
		//from the indices of the quad, compute the indices of the triangles
		unsigned int twoWidth = 2 * m_width;
		unsigned int offsetY = _z * twoWidth;
		unsigned int newId = offsetY + (_x * 2);
		_ids[0] = newId;
		_ids[1] = ++newId;
	}

	bool snHeightMap::isValidQuad(unsigned int _x, unsigned int _y) const
	{
		if (_y < 0 || _y >= m_length || _x < 0 || _x >= m_width)
			return false;
		
		return true;
	}
}