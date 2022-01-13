#include "NewtonPhysicsWorld.h"
#include "NewtonFixedDistanceConstraint.h"
#include "UrhoNewtonConversions.h"
#include "NewtonRigidBody.h"


#include "Urho3D/Scene/Component.h"
#include "Urho3D/Scene/Scene.h"
#include "Urho3D/Core/Context.h"
#include "Urho3D/IO/Log.h"


#include "ndNewton.h"







namespace Urho3D {




    NewtonFixedDistanceConstraint::NewtonFixedDistanceConstraint(Context* context) : NewtonConstraint(context)
    {
    }

    NewtonFixedDistanceConstraint::~NewtonFixedDistanceConstraint()
    {
    }

    void NewtonFixedDistanceConstraint::RegisterObject(Context* context)
    {
        context->RegisterFactory<NewtonFixedDistanceConstraint>(DEF_PHYSICS_CATEGORY.c_str());


        URHO3D_COPY_BASE_ATTRIBUTES(NewtonConstraint);



    }


    void NewtonFixedDistanceConstraint::buildConstraint()
{
        //get own body transform.
        ndVector pivot0 = UrhoToNewton(GetOwnBuildWorldFrame().Translation());
        ndVector pivot1 = UrhoToNewton(GetOtherBuildWorldFrame().Translation());
        

        newtonConstraint_ = new ndJointFixDistance(pivot0, pivot1, GetOwnNewtonBodyBuild(), GetOtherNewtonBodyBuild());

    }

}
