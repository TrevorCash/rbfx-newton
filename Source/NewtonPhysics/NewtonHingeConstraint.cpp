#include "NewtonHingeConstraint.h"
#include "NewtonPhysicsWorld.h"
#include "UrhoNewtonConversions.h"
#include "NewtonDebugDrawing.h"
#include "NewtonRigidBody.h"





#include "Urho3D/Core/Context.h"
#include "Urho3D/Scene/Component.h"
#include "Urho3D/Scene/Scene.h"
#include "Urho3D/Graphics/DebugRenderer.h"
#include "Urho3D/IO/Log.h"


#include "Newton.h"
#include "dMatrix.h"
#include "dCustomHinge.h"
#include "dCustomHingeActuator.h"
#include "dCustomMotor.h"
#include "dCustomJoint.h"



namespace Urho3D {

    const char* hingePoweredModeNames[] =
    {
        "NO_POWER",
        "MOTOR",
        "ACTUATOR",
        nullptr
    };


    NewtonHingeConstraint::NewtonHingeConstraint(Context* context) : NewtonConstraint(context)
    {

    }

    NewtonHingeConstraint::~NewtonHingeConstraint()
    {

    }

    void NewtonHingeConstraint::RegisterObject(Context* context)
    {
        context->RegisterFactory<NewtonHingeConstraint>(DEF_PHYSICS_CATEGORY.c_str());


        URHO3D_COPY_BASE_ATTRIBUTES(NewtonConstraint);

        URHO3D_ACCESSOR_ATTRIBUTE("Enable Limits", GetLimitsEnabled, SetEnableLimits, bool, true, AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("Angle Min", GetMinAngle, SetMinAngle, float, -45.0f, AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("Angle Max", GetMaxAngle, SetMaxAngle, float, 45.0f, AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("Friction", GetFriction, SetFriction, float, 0.0f, AM_DEFAULT);
        URHO3D_ENUM_ACCESSOR_ATTRIBUTE("Power Mode", GetPowerMode, SetPowerMode, PoweredMode, hingePoweredModeNames, PoweredMode::NO_POWER, AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("Max Torque", GetMaxTorque, SetMaxTorque, float, 10000.0f, AM_DEFAULT);

        URHO3D_ACCESSOR_ATTRIBUTE("Actuator Max Angular Rate", GetActuatorMaxAngularRate, SetActuatorMaxAngularRate, float, 1.0f, AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("Actuator Target Angle", GetActuatorTargetAngle, SetActuatorTargetAngle, float, 0.0f, AM_DEFAULT);

        URHO3D_ACCESSOR_ATTRIBUTE("Spring Damper Enable", GetNoPowerSpringDamper, SetNoPowerSpringDamper, bool, false, AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("Spring Coefficient", GetNoPowerSpringCoefficient, SetNoPowerSpringCoefficient, float, HINGE_CONSTRAINT_DEF_SPRING_COEF, AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("Spring Damper Coefficient", GetNoPowerDamperCoefficient, SetNoPowerDamperCoefficient, float, HINGE_CONSTRAINT_DEF_DAMPER_COEF, AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("Spring Damper Relaxation", GetNoPowerSpringDamperRelaxation, SetNoPowerSpringDamperRelaxation, float, HINGE_CONSTRAINT_DEF_RELAX, AM_DEFAULT);


    }

    void NewtonHingeConstraint::SetMinAngle(float minAngle)
    {
        if (minAngle_ != minAngle) {
            minAngle_ = minAngle;

            WakeBodies();

			if (newtonJoint_)
			{
				static_cast<dCustomHinge*>(newtonJoint_)->SetLimits(minAngle_ * dDegreeToRad, maxAngle_ * dDegreeToRad);
			}
            else
                MarkDirty();
        }
    }

    void NewtonHingeConstraint::SetMaxAngle(float maxAngle)
    {
        if (maxAngle_ != maxAngle) {
            maxAngle_ = maxAngle;
            WakeBodies();
            if (newtonJoint_) {
				static_cast<dCustomHinge*>(newtonJoint_)->SetLimits(minAngle_ * dDegreeToRad, maxAngle_ * dDegreeToRad);
            }
            else
                MarkDirty();
        }
    }

    void NewtonHingeConstraint::SetEnableLimits(bool enable)
    {
        if (enableLimits_ != enable) 
		{
            enableLimits_ = enable;
            WakeBodies();
            if (newtonJoint_) {
				static_cast<dCustomHinge*>(newtonJoint_)->EnableLimits(enableLimits_);
            }
            else
                MarkDirty();
        }
    }

    void NewtonHingeConstraint::SetFriction(float friction)
    {

        if (frictionTorque_ != friction) {
            frictionTorque_ = friction;
            WakeBodies();
            if (newtonJoint_) {
                if (powerMode_ == NO_POWER)
                    dynamic_cast<dCustomHinge*>(newtonJoint_)->SetFriction((frictionTorque_));
            }
            else
                MarkDirty();
        }
    }

    void NewtonHingeConstraint::SetMaxTorque(float torque)
    {
        if (maxTorque_ != torque)
        {
            maxTorque_ = torque;
            WakeBodies();
            if (newtonJoint_)
            {
                if (powerMode_ == ACTUATOR)
                    static_cast<dCustomHingeActuator*>(newtonJoint_)->SetMaxTorque((maxTorque_));
                else if (powerMode_ == MOTOR_SPEED )
                    static_cast<dCustomHinge*>(newtonJoint_)->SetFriction((maxTorque_));
            }
            else
                MarkDirty();
        }
    }

    void NewtonHingeConstraint::SetPowerMode(PoweredMode mode)
    {
        if (powerMode_ != mode) {
            powerMode_ = mode;
            MarkDirty();
        }
        else
            MarkDirty();
    }



    void NewtonHingeConstraint::SetActuatorMaxAngularRate(float rate)
    {
        if (maxAngularRate_ != rate)
        {
            maxAngularRate_ = rate;
            WakeBodies();
            if (newtonJoint_)
            {
                if (powerMode_ == ACTUATOR)
                    static_cast<dCustomHingeActuator*>(newtonJoint_)->SetAngularRate(maxAngularRate_);
            }
            else
                MarkDirty();
        }
    }

    void NewtonHingeConstraint::SetActuatorTargetAngle(float angle)
    {
        if (targetAngle_ != angle)
        {
            targetAngle_ = angle;
            WakeBodies();
            if (newtonJoint_)
            {
                if (powerMode_ == ACTUATOR)
                    static_cast<dCustomHingeActuator*>(newtonJoint_)->SetTargetAngle(targetAngle_* dDegreeToRad);
            }
            else
                MarkDirty();
        }
    }


    void NewtonHingeConstraint::SetMotorTargetAngularRate(float rate)
    {
        if (maxAngularRate_ != rate)
        {
            maxAngularRate_ = rate;
            WakeBodies();
            if (newtonJoint_)
            {
                if (powerMode_ == MOTOR_SPEED)
                {
                    static_cast<dCustomHinge*>(newtonJoint_)->EnableMotor(true, maxAngularRate_);
                }
            }
            else
                MarkDirty();
        }
    }


    void NewtonHingeConstraint::SetMotorTorque(float torque)
    {
        if (commandedTorque_ != torque)
        {
			commandedTorque_ = torque;
            WakeBodies();
            if (newtonJoint_)
            {
                if (powerMode_ == MOTOR_TORQUE)
                {
					float speed = 20000;

					if (commandedTorque_ < 0.0f)
						speed = -speed;

					static_cast<dCustomHinge*>(newtonJoint_)->EnableMotor(true, speed);
					
					static_cast<dCustomHinge*>(newtonJoint_)->SetFriction((commandedTorque_));
                }
            }
            else
                MarkDirty();
        }
    }



    void NewtonHingeConstraint::SetNoPowerSpringDamper(bool enable)
    {
        if (enableSpringDamper_ != enable)
        {
            enableSpringDamper_ = enable;

            if (newtonJoint_)
            {
                if (powerMode_ == NO_POWER)
                {
                    static_cast<dCustomHinge*>(newtonJoint_)->SetAsSpringDamper(enableSpringDamper_, springRelaxation_, springSpringCoef_, springDamperCoef_);
                }
            }
            else
                MarkDirty();

        }
    }



    void NewtonHingeConstraint::SetNoPowerSpringCoefficient(float springCoef)
    {
        if (springSpringCoef_ != springCoef)
        {
            springSpringCoef_ = springCoef;

            if (newtonJoint_)
            {
                if (powerMode_ == NO_POWER)
                {
                    static_cast<dCustomHinge*>(newtonJoint_)->SetAsSpringDamper(enableSpringDamper_, springRelaxation_, springSpringCoef_, springDamperCoef_);
                }
            }
            else
                MarkDirty();

        }
    }

    void NewtonHingeConstraint::SetNoPowerDamperCoefficient(float damperCoef)
    {
        if (springDamperCoef_ != damperCoef)
        {
            springDamperCoef_ = damperCoef;

            if (newtonJoint_)
            {
                if (powerMode_ == NO_POWER)
                {
                    static_cast<dCustomHinge*>(newtonJoint_)->SetAsSpringDamper(enableSpringDamper_, springRelaxation_, springSpringCoef_, springDamperCoef_);
                }
            }
            else
                MarkDirty();

        }
    }

    void NewtonHingeConstraint::SetNoPowerSpringDamperRelaxation(float relaxation)
    {
        if (springRelaxation_ != relaxation)
        {
            springRelaxation_ = relaxation;

            if (newtonJoint_)
            {
                if (powerMode_ == NO_POWER)
                {
                    static_cast<dCustomHinge*>(newtonJoint_)->SetAsSpringDamper(enableSpringDamper_, springRelaxation_, springSpringCoef_, springDamperCoef_);
                }
            }
            else
                MarkDirty();

        }
    }

    float NewtonHingeConstraint::GetCurrentAngularRate()
    {
        if (newtonJoint_)
        {
            return static_cast<dCustomHinge*>(newtonJoint_)->GetJointOmega();
        }
        return 0.0f;
    }

    float NewtonHingeConstraint::GetCurrentAngle()
    {
        if (newtonJoint_)
        {
            return static_cast<dCustomHinge*>(newtonJoint_)->GetJointAngle();
        }
        return 0.0f;
    }

    void NewtonHingeConstraint::DrawDebugGeometry(DebugRenderer* debug, bool depthTest)
    {
        NewtonConstraint::DrawDebugGeometry(debug, depthTest);
    }

    void NewtonHingeConstraint::buildConstraint()
    {
        // Create a dCustomHinge

        if (powerMode_ == ACTUATOR)
        {
            newtonJoint_ = new dCustomHingeActuator(UrhoToNewton(GetOwnBuildWorldFrame()), maxAngularRate_, minAngle_ * dDegreeToRad, maxAngle_ * dDegreeToRad, GetOwnNewtonBodyBuild(), GetOtherNewtonBodyBuild());
        }
        else if (powerMode_ == MOTOR_SPEED)
        {
            newtonJoint_ = new dCustomHinge(UrhoToNewton(GetOwnBuildWorldFrame()), UrhoToNewton(GetOtherBuildWorldFrame()), GetOwnNewtonBodyBuild(), GetOtherNewtonBodyBuild());
        }
		else if (powerMode_ == MOTOR_TORQUE)
		{
			newtonJoint_ = new dCustomHinge(UrhoToNewton(GetOwnBuildWorldFrame()), UrhoToNewton(GetOtherBuildWorldFrame()), GetOwnNewtonBodyBuild(), GetOtherNewtonBodyBuild());
		}
        else
        {
            newtonJoint_ = new dCustomHinge(UrhoToNewton(GetOwnBuildWorldFrame()), UrhoToNewton(GetOtherBuildWorldFrame()), GetOwnNewtonBodyBuild(), GetOtherNewtonBodyBuild());
        }


    }

    bool NewtonHingeConstraint::applyAllJointParams()
    {
        if (!NewtonConstraint::applyAllJointParams())
            return false;

        if (powerMode_ == ACTUATOR)
        {
            //static_cast<dCustomHingeActuator*>(newtonJoint_)->EnableLimits(enableLimits_); this breaks.
            static_cast<dCustomHingeActuator*>(newtonJoint_)->SetLimits(minAngle_ * dDegreeToRad, maxAngle_ * dDegreeToRad);
            static_cast<dCustomHingeActuator*>(newtonJoint_)->SetTargetAngle(targetAngle_* dDegreeToRad);
            static_cast<dCustomHingeActuator*>(newtonJoint_)->SetMaxTorque((maxTorque_));
            static_cast<dCustomHingeActuator*>(newtonJoint_)->SetAngularRate(maxAngularRate_);
        }
        else if (powerMode_ == MOTOR_SPEED)
        {
			static_cast<dCustomHinge*>(newtonJoint_)->EnableLimits(enableLimits_);
            static_cast<dCustomHinge*>(newtonJoint_)->SetFriction((maxTorque_));
            static_cast<dCustomHinge*>(newtonJoint_)->EnableMotor(true, maxAngularRate_);
        }
        else if (powerMode_ == MOTOR_TORQUE)
        {
			static_cast<dCustomHinge*>(newtonJoint_)->EnableLimits(enableLimits_);
			static_cast<dCustomHinge*>(newtonJoint_)->EnableMotor(true, 20000);
			static_cast<dCustomHinge*>(newtonJoint_)->SetFriction((commandedTorque_));


        }
        else if(powerMode_ == NO_POWER)
        {
            static_cast<dCustomHinge*>(newtonJoint_)->EnableLimits(enableLimits_);
            static_cast<dCustomHinge*>(newtonJoint_)->SetLimits(minAngle_ * dDegreeToRad, maxAngle_ * dDegreeToRad);
            static_cast<dCustomHinge*>(newtonJoint_)->SetFriction((frictionTorque_));
            static_cast<dCustomHinge*>(newtonJoint_)->SetAsSpringDamper(enableSpringDamper_, springRelaxation_, springSpringCoef_, springDamperCoef_);
        }


        return true;
    }



}
