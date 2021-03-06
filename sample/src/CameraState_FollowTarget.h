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

#ifndef CAMERA_STATE_FOLLOW_TARGET_H
#define CAMERA_STATE_FOLLOW_TARGET_H

#include "IState.h"
using namespace Devil::FSM;

#include "snVec.h"

using Supernova::snVec;
#include "snGlobals.h"

#include "VecLinearDamper.h"

namespace Devil
{
	class IWorldEntity;
	class EntityCamera;

	SN_ALIGN class CameraState_FollowTarget : public IState
	{
	private:
		EntityCamera* m_camera;

		//The target to follow
		IWorldEntity const * m_target;

		//Distance from the target to the camera.
		float m_distance;

		//Height of the camera above the target.
		float m_height;

		//The forward vector of the camera.
		snVec m_forward;

		//Damper used to smooth the camera position
		VecLinearDamper m_positionDamper;

		//Damper used to smooth the camera look at.
		VecLinearDamper m_lookAtDamper;

		//Rotation of the camera around the Y axis
		float m_angleY;

	public:
		CameraState_FollowTarget(EntityCamera* _camera);
		~CameraState_FollowTarget();

		void enter();

		void execute();

		void exit();

		void* operator new(size_t _count);

		void operator delete(void* _p);

		//Prepare the state
		// _target : the entity the camera has to follow.
		// _distance : the distance between the target and the camera.
		// _height : the distance of the camera above the target.
		void setup(IWorldEntity const * const _target, float _distance, float _height);
	};
}
#endif
//ifndef CAMERA_STATE_FOLLOW_TARGET_H