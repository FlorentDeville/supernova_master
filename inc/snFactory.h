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

#ifndef SN_FACTORY_H
#define SN_FACTORY_H

#include "snGlobals.h"

#include <vector>
using std::vector;

namespace Supernova
{

	class snScene;

	//Main entry point of Supernova.
	class SN_ALIGN snFactory
	{
	private:

		//Singleton
		static snFactory* m_instance;

		//List of scenes.
		vector<snScene*> m_scenes;

	private:
		//Constructor
		snFactory();

		//Destructor
		virtual ~snFactory();

	public:
		//Return the unique instance of the snFactory.
		static snFactory* getInstance();

		//Initialize the factory
		bool initialize();

		//Delete all allocations made by the physics engine.
		bool clean();

		//Create an empty scene.
		void createScene(snScene** _newScene, int& _sceneId);

		//Delete a scene and all its actors.
		void deleteScene(int _sceneId);

		//Get a scene from its id. Return null if the scene can't be found.
		snScene* getScene(int _sceneId);
	};

//Provide quick access to the factory.
#define SUPERNOVA ::Supernova::snFactory::getInstance()

}
#endif //SN_FACTORY_H