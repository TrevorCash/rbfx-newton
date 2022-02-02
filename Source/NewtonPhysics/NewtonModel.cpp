#include "NewtonModel.h"

#include "NewtonConstraint.h"
#include "NewtonPhysicsWorld.h"
#include "NewtonRigidBody.h"


#include "Urho3D/Scene/Scene.h"

namespace  Urho3D
{
	void NewtonModel::RegisterObject(Context* context)
	{
		context->RegisterFactory<NewtonModel>(DEF_PHYSICS_CATEGORY.c_str());

	}

	NewtonModel::NewtonModel(Context* context) : Component(context)
	{
	}

	void NewtonModel::Grow(NewtonConstraint* constraint)
	{
        constraint->model_ = this;
        ea::vector<NewtonRigidBody*> bodies;
        ea::vector<NewtonConstraint*> constraints;
        physicsWorld_->GetConnectedPhysicsComponents(constraint->GetOwnBody(), bodies, constraints);

        for (auto* body : bodies)
            body->model_ = this;

        for (auto* constraint : constraints)
            constraint->model_ = this;

	}

	inline void NewtonModel::OnSceneSet(Scene* scene)
	{
		Component::OnSceneSet(scene);
        if (scene)
        {
            ///auto create physics world
            NewtonPhysicsWorld* physicsWorld_ = WeakPtr<NewtonPhysicsWorld>(scene->GetComponent<NewtonPhysicsWorld>());

            physicsWorld_->addModel(this);
            scene->AddListener(this);
        }
        else
        {
            //TODO - remove
            if (physicsWorld_) {
                physicsWorld_->WaitForUpdateFinished();
            }
        }
	}
}

