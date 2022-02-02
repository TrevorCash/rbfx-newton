#include "NewtonPhysicsWorld.h"
#include "NewtonKinematicsJoint.h"
#include "UrhoNewtonConversions.h"
#include "NewtonRigidBody.h"
#include "NewtonPhysicsEvents.h"
#include "NewtonConstraint.h"

#include "Urho3D/Graphics/DebugRenderer.h"
#include "Urho3D/IO/Log.h"
#include "Urho3D/Core/Context.h"


#include "Urho3D/Core/CoreEvents.h"





namespace Urho3D {




    NewtonKinematicsControllerConstraint::NewtonKinematicsControllerConstraint(Context* context) : NewtonConstraint(context)
    {
        SubscribeToEvent(E_UPDATE, URHO3D_HANDLER(NewtonKinematicsControllerConstraint, HandleUpdate));
    }

    NewtonKinematicsControllerConstraint::~NewtonKinematicsControllerConstraint()
    {

    }

    void NewtonKinematicsControllerConstraint::RegisterObject(Context* context)
    {
        context->RegisterFactory<NewtonKinematicsControllerConstraint>(DEF_PHYSICS_CATEGORY.c_str());

        URHO3D_COPY_BASE_ATTRIBUTES(NewtonConstraint);
    }

    void NewtonKinematicsControllerConstraint::DrawDebugGeometry(DebugRenderer* debug, bool depthTest)
    {
        NewtonConstraint::DrawDebugGeometry(debug, depthTest);
    }

    void NewtonKinematicsControllerConstraint::SetLinearFrictionalAcceleration(float friction)
    {
        if (linearFrictionalAcceleration != friction) {
            linearFrictionalAcceleration = friction;
            if(newtonConstraint_)
                updateFrictions();
        }
    }

    void NewtonKinematicsControllerConstraint::SetAngularFrictionalAcceleration(float friction)
    {
        if (angularFrictionalAcceleration != friction) {
            angularFrictionalAcceleration = friction;
            if (newtonConstraint_)
                updateFrictions();
        }
    }

    void NewtonKinematicsControllerConstraint::SetConstrainRotation(bool enable)
    {
        if (constrainRotation_ != enable)
        {
            constrainRotation_ = enable;
			if (newtonConstraint_)
			{
				if (constrainRotation_)
				{
                    static_cast<ndJointKinematicController*>(newtonConstraint_)->SetControlMode(ndJointKinematicController::m_full6dof);
				}
				else
				{
					static_cast<ndJointKinematicController*>(newtonConstraint_)->SetControlMode(ndJointKinematicController::m_linear);
				}
			}
        }
    }

    void NewtonKinematicsControllerConstraint::SetLimitRotationalVelocity(bool enable)
    {
        if (limitRotationalVelocity_ != enable)
        {
            limitRotationalVelocity_ = enable;
            //if(newtonJoint_)
            //    static_cast<dCustomKinematicController*>(newtonJoint_)->SetLimitRotationVelocity(limitRotationalVelocity_);
        }
    }


    void NewtonKinematicsControllerConstraint::SetOtherPosition(const Vector3& position)
    {
		otherPosition_ = position;
    }

    void NewtonKinematicsControllerConstraint::SetOtherRotation(const Quaternion& rotation)
    {
		otherRotation_ = rotation;

    }

  

    void NewtonKinematicsControllerConstraint::buildConstraint()
    {
        newtonConstraint_ = new ndJointKinematicController(GetOtherNewtonBodyBuild()->GetAsBodyKinematic(), GetOwnNewtonBodyBuild()->GetAsBodyKinematic(), UrhoToNewton(GetOwnBuildWorldFrame()));
        
		
		
		//#todo support all control modes
		if (constrainRotation_)
		{
			static_cast<ndJointKinematicController*>(newtonConstraint_)->SetControlMode(ndJointKinematicController::m_linear);
		}
		else
		{
			static_cast<ndJointKinematicController*>(newtonConstraint_)->SetControlMode(ndJointKinematicController::m_full6dof);
		}
		
		updateFrictions();
        //static_cast<dCustomKinematicController*>(newtonJoint_)->SetLimitRotationVelocity(limitRotationalVelocity_);
    }

    void NewtonKinematicsControllerConstraint::updateTarget()
    {
        if (newtonConstraint_) {
            static_cast<ndJointKinematicController*>(newtonConstraint_)->SetTargetMatrix(UrhoToNewton(GetOtherWorldFrame()));
        }
    }

    void NewtonKinematicsControllerConstraint::updateFrictions()
    {

        ndFloat32 Ixx;
        ndFloat32 Iyy;
        ndFloat32 Izz;
        ndFloat32 mass;
		ownBody_->GetNewtonBody()->GetAsBodyDynamic()->GetMassMatrix(Ixx, Iyy, Izz, mass);

        const ndFloat32 inertia = dMax(Izz, dMax(Ixx, Iyy));


        static_cast<ndJointKinematicController*>(newtonConstraint_)->SetMaxLinearFriction(mass * linearFrictionalAcceleration);
        static_cast<ndJointKinematicController*>(newtonConstraint_)->SetMaxAngularFriction(inertia * angularFrictionalAcceleration);
    }

    void NewtonKinematicsControllerConstraint::HandleUpdate(StringHash event, VariantMap& eventData)
    {
        updateTarget();

		if (ownBody_->GetIsKinematic()) {

			Matrix3x4 ownLocalTransform = Matrix3x4(position_, rotation_, 1.0f);


			ownBody_->SetWorldTransform(Matrix3x4(otherPosition_, otherRotation_, 1.0f) * ownLocalTransform.Inverse() );

		}
    }

}
