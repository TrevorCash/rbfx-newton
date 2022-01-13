#include "NewtonFullyFixedConstraint.h"
#include "NewtonPhysicsWorld.h"
#include "UrhoNewtonConversions.h"
#include "NewtonRigidBody.h"


#include "Urho3D/Scene/Component.h"
#include "Urho3D/Scene/Scene.h"
#include "Urho3D/Core/Context.h"




namespace Urho3D {


    NewtonFullyFixedConstraint::NewtonFullyFixedConstraint(Context* context) : NewtonConstraint(context)
    {

    }

    NewtonFullyFixedConstraint::~NewtonFullyFixedConstraint()
    {

    }

    void NewtonFullyFixedConstraint::RegisterObject(Context* context)
    {
        context->RegisterFactory<NewtonFullyFixedConstraint>(DEF_PHYSICS_CATEGORY.c_str());

        URHO3D_COPY_BASE_ATTRIBUTES(NewtonConstraint);
    }

    void NewtonFullyFixedConstraint::buildConstraint()
    {

        Matrix3x4 ownFrame = GetOwnBuildWorldFrame();
        Matrix3x4 otherFrame = GetOtherBuildWorldFrame();


        newtonConstraint_ = new dCustomSixdof(UrhoToNewton(ownFrame), UrhoToNewton(otherFrame), GetOwnNewtonBodyBuild(), GetOtherNewtonBodyBuild());

    }

}
