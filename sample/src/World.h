#ifndef WORLD_H
#define WORLD_H

#include <vector>

#include <DirectXMath.h>
using namespace DirectX;


namespace Supernova
{
	class snFixedConstraint;
}

using namespace Supernova;
namespace Devil
{
	class IWorldEntity;
	class EntitySphere;
	class EntityBox;
	class EntityPlan;
	class EntityCollisionPoint;
	class EntityCamera;
	class EntityFixedConstraint;

	class Input;

	class World
	{
	private:
		static World* m_Instance;

		std::vector<IWorldEntity*> m_EntityList;

		EntityCamera* m_camera;

		Input* m_input;

		EntityCollisionPoint* m_collisionPoint;

	public:
		virtual ~World();

		static World* getInstance();
		static void shutdown();

		bool initialize(Input* _input);

		EntitySphere* createSphere(float);
		EntityBox* createBox(const XMFLOAT3&);
		EntityBox* createBox(const XMFLOAT3& _size, const XMFLOAT4& _color);
		EntityPlan* createPlan(const XMFLOAT2& _size, const XMFLOAT4& _color);
		EntityCamera* createCamera(const XMVECTOR& _position, const XMVECTOR& _lookAt, const XMVECTOR& _up);
		EntityFixedConstraint* createFixedConstraint(const snFixedConstraint* _constraint);

		//Delete all entities from the world.
		void clearWorld();

		void update();
		void render();

		Input* getInput() const;
		EntityCamera* getCamera() const;

		void toggleCollisionPointActivation();
		void activateCollisionPoint();
		void deactivateCollisionPoint();
	private:
		World();

		EntityCollisionPoint* createCollisionPoint(float _diameter);
	};

#define WORLD World::getInstance()
}

#endif