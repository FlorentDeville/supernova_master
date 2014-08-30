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
#ifndef ENTITY_CAMERA_H
#define ENTITY_CAMERA_H

#include "IWorldEntity.h"

#include "FSMRunner.h"
using namespace Devil::FSM;

namespace Devil
{
	class Camera;
	class CameraState_FollowTarget;

	class EntityCamera : public IWorldEntity
	{
	private:
		//XMVECTOR m_position;
		XMVECTOR m_lookAt;
		XMVECTOR m_up;

		Camera* m_gfxCamera;
		
		FSMRunner m_fsmRunner;

		//Pointer to the state object FollowTarget.
		CameraState_FollowTarget* m_stateFollowTarget;

	public:
		EntityCamera();
		~EntityCamera();

		bool initialize(const XMVECTOR& _position, const XMVECTOR& _lookAt, const XMVECTOR& _up);

		void update();
		void render();

		void setLookAt(const XMVECTOR& _lookAt);
		void setUp(const XMVECTOR& _up);

		//Get the camera look at vector
		const XMVECTOR& getLookAt() const;
		const XMVECTOR& getUp() const;

		void* operator new(size_t _count);

		void operator delete(void* _p);

		//Change the current camera mode and set it to the follow target mode.
		// _target : the entity to follow.
		// _distance : the distance between the target and the camera.
		// _height : the distance on the y axis between the target and the camera.
		void setCameraModeFollowTarget(IWorldEntity const * const _target, float _distance, float _height);

		//change the current camera mode to the free camera mode.
		void setCameraModeFree();
	};

}
#endif