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

#include "snArbiterKey.h"

namespace Supernova
{
	snArbiterKey::snArbiterKey(snObjectId collider1, snObjectId collider2)
	{
		if(collider1 < collider2)
		{
			shapeId[0] = collider1;
			shapeId[1] = collider2;
		}
		else
		{
			shapeId[0] = collider2;
			shapeId[1] = collider1;
		}
	}


	bool operator < (const snArbiterKey& _k1, const snArbiterKey& _k2)
	{
		if(_k1.shapeId[0] < _k2.shapeId[0])
			return true;

		return false;
	}

	bool operator == (const snArbiterKey& _k1, const snArbiterKey& _k2)
	{
		if(_k1.shapeId[0] == _k2.shapeId[0] && _k1.shapeId[1] == _k2.shapeId[1])
			return true;

		return false;
	}

	bool operator != (const snArbiterKey& _k1, const snArbiterKey& _k2)
	{
		if(_k1.shapeId[0] == _k2.shapeId[0] && _k1.shapeId[1] == _k2.shapeId[1])
			return false;

		return true;
	}
}