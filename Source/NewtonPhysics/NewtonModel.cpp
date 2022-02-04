#include "NewtonModel.h"

#include "NewtonConstraint.h"
#include "NewtonHingeConstraint.h"
#include "NewtonPhysicsWorld.h"
#include "NewtonRigidBody.h"

#include "Urho3D/Scene/Scene.h"

#include "ndNewton.h"


namespace  Urho3D
{
	void NewtonModel::RegisterObject(Context* context)
	{
		context->RegisterFactory<NewtonModel>(DEF_PHYSICS_CATEGORY.c_str());

	}

	NewtonModel::NewtonModel(Context* context) : Component(context)
	{
	}

	void NewtonModel::GrowFrom(NewtonConstraint* constraint)
	{
        constraint->model_ = this;
        bodies.clear();
        constraints.clear();

        physicsWorld_->GetConnectedPhysicsComponents(constraint->GetOwnBody(), bodies, constraints);

        for (auto* body : bodies)
            body->model_ = this;

        for (auto* constraint : constraints)
            constraint->model_ = this;

	}

	void NewtonModel::Grow()
	{
        GrowFrom(constraints.front());
	}



    void NewtonModel::freeModel()
    {
        if (newtonModel_ != nullptr) 
        {
            physicsWorld_->GetNewtonWorld()->RemoveModel(newtonModel_);
            physicsWorld_->freeModelQueue_.push_back(newtonModel_);
            newtonModel_ = nullptr;
        }
    }

    void NewtonModel::reBuild()
    {
        freeModel();
        newtonModel_ = new NewtonModelHandler();
        newtonModel_->model_ = this;
        physicsWorld_->GetNewtonWorld()->AddModel(newtonModel_);
    }





















    void NewtonModel::PreSolveComputations(ndWorld* const world, ndFloat32 timestep)
    {
       // NewtonRigidBody* base = nullptr;



       // for(auto* body : bodies)
       // {
       //     if (body->GetNode()->GetName() == "RobotBase")
       //         base = body;
       // }



       // ndBodyKinematic* sceneBody = base->GetScene()->GetComponent<NewtonRigidBody>()->GetNewtonBody()->GetAsBodyKinematic();

       // //ndJointKinematicController* endEffector = new ndJointKinematicController(sceneBody, ,)
       // //end

       // 

       // ndSkeletonContainer* const skeleton = base->GetNewtonBody()->GetAsBodyKinematic()->GetSkeleton();
       // ndSkeletonImmediateSolver solve;
       // solve.Solve(skeleton, world, timestep);

       // // use solver result to set joint motors
       // //for(int i = 0; i < solve.m_internalForces.GetCount(); i++)
       // //{
       //     ndVector angular = solve.m_internalForces[0].m_angular;
       //     ndVector linear = solve.m_internalForces[0].m_linear;

       //     ea::vector<NewtonConstraint*> constraint;
       //     base->GetConnectedContraints(constraint);

       //     dynamic_cast<NewtonHingeConstraint*>(constraint[0])->SetCommandedTorque(angular.GetX());
       //     


       //// }



    }

    inline void NewtonModel::OnSceneSet(Scene* scene)
	{
		Component::OnSceneSet(scene);
        if (scene)
        {
            ///auto create physics world
            physicsWorld_ = WeakPtr<NewtonPhysicsWorld>(scene->GetComponent<NewtonPhysicsWorld>());

            physicsWorld_->addModel(this);
            scene->AddListener(this);
        }
        else
        {
            if (!physicsWorld_.Expired())
				physicsWorld_->removeModel(this);
        }
	}




















    NewtonModelHandler::NewtonModelHandler() : ndModel()
    {
    }

    void NewtonModelHandler::Update(ndWorld* const world, ndFloat32 timestep)
    {
        ndModel::Update(world, timestep);
        model_->PreSolveComputations(world, timestep);


    }

    void NewtonModelHandler::PostUpdate(ndWorld* const world, ndFloat32 timestep)
    {
        ndModel::PostUpdate(world, timestep);
    }

    void NewtonModelHandler::PostTransformUpdate(ndWorld* const world, ndFloat32 timestep)
    {
        ndModel::PostTransformUpdate(world, timestep);
    }

















}

