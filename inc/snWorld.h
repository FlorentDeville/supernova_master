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

#ifndef SN_WORLD_H
#define SN_WORLD_H

#include "snGlobals.h"
#include "snObject.h"
#include "snVec.h"

#include <vector>
using std::vector;

#include <map>
using std::map;

namespace Supernova
{
	class snObject;
	class snScene;
	class snActorDynamic;
	class snActorStatic;

	template<class C> class snHandle
	{
	protected:

		//The id of the object in the look up table of Supernova.
		snObjectId m_id;

	public:
		snHandle(){ m_id = 0; }

		snHandle(snObjectId _id) { m_id = _id; }

		virtual ~snHandle(){}

		C* const getPtr() const { return static_cast<C* const>(snWorld::getInstance()->getObject(m_id)); }

		snObjectId getId() const { return m_id; }

		bool isValid() const { return getPtr() == 0 ? false : true; }

		C* const operator->() const { return getPtr(); }
	};

	typedef snHandle<snScene> snhScene;					//Handle for a scene.
	typedef snHandle<snActorStatic> snhActorStatic;		//Handle for a static actor.
	typedef snHandle<snActorDynamic> snhActorDynamic;	//Handle for a dynamic actor.

	//Main entry point of Supernova.
	class SN_ALIGN snWorld
	{
	private:

		//Singleton
		static snWorld* m_instance;

		//Store the next key to use to identify uniquely an object created by the World.
		//It goes from 0 to 2^32 = 4 294 967 295.
		unsigned int m_key;

		//Map to store pointers to the created objects with their keys.
		map<snObjectId, snObject*> m_lookUpTable;

	private:
		//Constructor
		snWorld();

		//Destructor
		virtual ~snWorld();

	public:
		//Return the unique instance of the snFactory.
		static snWorld* getInstance();

		//Initialize the factory
		bool initialize();

		//Delete all allocations made by the physics engine.
		bool clean();

		//Return a pointer to an object based on its id. Returns 0 if the id is invalid.
		// _id : id of an object.
		void* getObject(snObjectId _id) const;

		//Create an empty scene and return an handle to it.
		// remarks : not thread safe.
		snhScene createScene();

		//Create a new actor.
		// return : a handle to the newly created dynamic actor.
		snhActorDynamic createActorDynamic();

		//Create a new static actor.
		// _position : the position of the static actor.
		// _orientation : the orientation as a quaternion of the static actor.
		// return : a handle to the newly created static actor.
		snhActorStatic createActorStatic(const snVec& _position, const snVec& _orientation);

		//Remove an object from the world.
		// _id : id of the object to remove from the world.
		// remarks : this method won't delete any data and the object will still exists in the engine.
		//           Any handle on the object will become invalid.
		void removeObject(snObjectId _id);

		//Delete a scene identifed by a handle.
		// _scene : handle of the scene to delete.
		// remarks : every actors and colliders from the scene will be deleted as well.
		void deleteScene(snhScene _scene);

		//Delete a scene identified by an object id.
		// _id : id of the object to delete.
		void deleteScene(snObjectId _id);

		//Overridden new operator to create scene with correct alignement.
		void* operator new(size_t _count);

		//Overridden delete operator to delete using the correct alignement.
		void operator delete(void* _p);
	};

//Provide quick access to the factory.
#define SUPERNOVA ::Supernova::snWorld::getInstance()

}
#endif //ifndef SN_WORLD_H