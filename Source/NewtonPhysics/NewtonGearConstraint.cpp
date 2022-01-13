#include "NewtonGearConstraint.h"
#include "NewtonPhysicsWorld.h"
#include "UrhoNewtonConversions.h"
#include "NewtonDebugDrawing.h"
#include "NewtonRigidBody.h"





#include "Urho3D/Core/Context.h"
#include "Urho3D/Scene/Component.h"
#include "Urho3D/Scene/Scene.h"
#include "Urho3D/Graphics/DebugRenderer.h"
#include "Urho3D/IO/Log.h"


#include "ndNewton.h"



namespace Urho3D {



	NewtonGearConstraint::NewtonGearConstraint(Context* context) : NewtonConstraint(context)
	{

	}

	NewtonGearConstraint::~NewtonGearConstraint()
	{

	}

	void NewtonGearConstraint::RegisterObject(Context* context)
	{
		context->RegisterFactory<NewtonGearConstraint>(DEF_PHYSICS_CATEGORY.c_str());


		URHO3D_COPY_BASE_ATTRIBUTES(NewtonConstraint);

		URHO3D_ACCESSOR_ATTRIBUTE("Ratio", GetGearRatio, SetGearRatio, float, 1.0f, AM_DEFAULT);
	}

	void NewtonGearConstraint::DrawDebugGeometry(DebugRenderer* debug, bool depthTest)
	{

	}

	void NewtonGearConstraint::buildConstraint()
	{
		//newtonConstraint_ = new ndJointGear(ratio_, UrhoToNewton(GetOwnBuildWorldFrame().Rotation() * Vector3::RIGHT), 
		//	UrhoToNewton(GetOtherBuildWorldFrame().Rotation() *  Vector3::RIGHT), GetOwnNewtonBodyBuild(), GetOtherNewtonBodyBuild());
	
	
	}

	bool NewtonGearConstraint::applyAllJointParams()
	{

		return true;
	}

}
