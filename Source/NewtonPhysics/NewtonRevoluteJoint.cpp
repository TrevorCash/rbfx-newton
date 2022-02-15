#include "NewtonRevoluteJoint.h"
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

    const char* hingePoweredModeNames[] =
    {
        "NO_POWER",
        "MOTOR",
        "ACTUATOR",
        nullptr
    };


    NewtonRevoluteJoint::NewtonRevoluteJoint(Context* context) : NewtonConstraint(context)
    {

    }

    NewtonRevoluteJoint::~NewtonRevoluteJoint()
    {

    }

    void NewtonRevoluteJoint::RegisterObject(Context* context)
    {
        context->RegisterFactory<NewtonRevoluteJoint>(DEF_PHYSICS_CATEGORY.c_str());
        URHO3D_COPY_BASE_ATTRIBUTES(NewtonConstraint);

        URHO3D_ACCESSOR_ATTRIBUTE("Enable Limits", GetHingeLimitsEnabled, SetEnableHingeLimits, bool, true, AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("Angle Min", GetMinAngle, SetMinAngle, float, -45.0f, AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("Angle Max", GetMaxAngle, SetMaxAngle, float, 45.0f, AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("Torque", GetCommandedTorque, SetCommandedTorque, float, 0.0f, AM_DEFAULT);
    }

    void NewtonRevoluteJoint::SetMinAngle(float minAngle)
    {
        if (minAngle_ != minAngle) {
            minAngle_ = minAngle;

            WakeBodies();

			if (newtonConstraint_)
			{
                static_cast<ndRevoluteJoint*>(newtonConstraint_)->m_minLimit = minAngle_ * ndDegreeToRad;
			}
            else
                MarkDirty();
        }
    }

    void NewtonRevoluteJoint::SetMaxAngle(float maxAngle)
    {
        if (maxAngle_ != maxAngle) {
            maxAngle_ = maxAngle;
            WakeBodies();
            if (newtonConstraint_) {
                static_cast<ndRevoluteJoint*>(newtonConstraint_)->m_maxLimit = maxAngle_ * ndDegreeToRad;
            }
            else
                MarkDirty();
        }
    }

    void NewtonRevoluteJoint::SetEnableHingeLimits(bool enable)
    {
        if (enableHingeLimits_ != enable) 
		{
            enableHingeLimits_ = enable;
            WakeBodies();
            if (newtonConstraint_) {
                static_cast<ndRevoluteJoint*>(newtonConstraint_)->m_hasRotationLimits = enableHingeLimits_;

            }
            else
                MarkDirty();
        }
    }

    void NewtonRevoluteJoint::SetEnableOffsetLimits(bool enable)
    {
        if (enableOffsetLimits_ != enable)
        {
            enableOffsetLimits_ = enable;
            WakeBodies();
            if (newtonConstraint_) {
                static_cast<ndRevoluteJoint*>(newtonConstraint_)->m_hasOffsetLimits = enableOffsetLimits_;

            }
            else
                MarkDirty();
        }
    }


    void NewtonRevoluteJoint::SetCommandedTorque(float torque)
    {
        if (commandedTorque_ != torque)
        {
            commandedTorque_ = torque;
            WakeBodies();
            if (newtonConstraint_)
            {
            	static_cast<ndRevoluteJoint*>(newtonConstraint_)->SetTorque(-commandedTorque_);
            }
            else
                MarkDirty();
        }
    }

    void NewtonRevoluteJoint::SetFrictionCoef(float frictionCoef)
    {
        if (frictionCoef_ != frictionCoef)
        {
            frictionCoef_ = frictionCoef;
            WakeBodies();
            if (newtonConstraint_)
            {
                static_cast<ndRevoluteJoint*>(newtonConstraint_)->m_internalFrictionCoef = frictionCoef_;
            }
            else
                MarkDirty();
        }
    }

    float NewtonRevoluteJoint::GetAngle()
    {
        if (newtonConstraint_)
        {
        	return static_cast<ndRevoluteJoint*>(newtonConstraint_)->m_angle * ndRadToDegree;
        }
        return 0.0f;
    }

    float NewtonRevoluteJoint::GetHingeAngularVelocity() const
    {
        if (newtonConstraint_)
        {
            //neg because newton to urho.
            return 1.0f*static_cast<ndRevoluteJoint*>(newtonConstraint_)->m_omega;
        }
        return 0.0f;
    }


    void NewtonRevoluteJoint::DrawDebugGeometry(DebugRenderer* debug, bool depthTest)
    {
        NewtonConstraint::DrawDebugGeometry(debug, depthTest);

        if (enableHingeLimits_) 
        {
            constexpr float lenScale = 4.0f;
            float minIndicator = lenScale * minAngle_ / 360.0f;
            float maxIndicator = lenScale * maxAngle_ / 360.0f;
            float curIndictor = lenScale * GetAngle() / 360.0f;

            float limitSize = maxIndicator / lenScale - minIndicator / lenScale;
            limitSize = Clamp(limitSize, 0.1f, 0.5f);


            debug->AddLine(GetOwnWorldFrame() * Vector3(minIndicator, 0, 0), GetOwnWorldFrame().Translation(), Color::RED, depthTest);
            debug->AddLine(GetOwnWorldFrame().Translation(), GetOwnWorldFrame() * Vector3(maxIndicator, 0, 0), Color::BLUE, depthTest);

            Vector3 a = GetOwnWorldFrame() * (Vector3(0, 1, -1) * limitSize + Vector3(minIndicator, 0, 0));
            Vector3 b = GetOwnWorldFrame() * (Vector3(0, 1, 1) * limitSize + Vector3(minIndicator, 0, 0));
            Vector3 c = GetOwnWorldFrame() * (Vector3(0, -1, 1) * limitSize + Vector3(minIndicator, 0, 0));
            Vector3 d = GetOwnWorldFrame() * (Vector3(0, -1, -1) * limitSize + Vector3(minIndicator, 0, 0));

            Vector3 a2 = GetOwnWorldFrame() * (Vector3(0, 1, -1) * limitSize + Vector3(maxIndicator, 0, 0));
            Vector3 b2 = GetOwnWorldFrame() * (Vector3(0, 1, 1) * limitSize + Vector3(maxIndicator, 0, 0));
            Vector3 c2 = GetOwnWorldFrame() * (Vector3(0, -1, 1) * limitSize + Vector3(maxIndicator, 0, 0));
            Vector3 d2 = GetOwnWorldFrame() * (Vector3(0, -1, -1) * limitSize + Vector3(maxIndicator, 0, 0));


            Vector3 a3 = GetOwnWorldFrame() * (Vector3(0, 1, -1) * limitSize * 0.5f + Vector3(curIndictor, 0, 0));
            Vector3 b3 = GetOwnWorldFrame() * (Vector3(0, 1, 1) * limitSize * 0.5f + Vector3(curIndictor, 0, 0));
            Vector3 c3 = GetOwnWorldFrame() * (Vector3(0, -1, 1) * limitSize * 0.5f + Vector3(curIndictor, 0, 0));
            Vector3 d3 = GetOwnWorldFrame() * (Vector3(0, -1, -1) * limitSize * 0.5f + Vector3(curIndictor, 0, 0));



            Color col1 = Color::RED;
            col1.a_ = 0.5f;
            Color col2 = Color::BLUE;
            col2.a_ = 0.5f;
            Color col3 = Color::GREEN;
            col3.a_ = 0.5f;
            debug->AddPolygon(a, b, c, d, col1, depthTest);
            debug->AddPolygon(a2, b2, c2, d2, col2, depthTest);
            debug->AddPolygon(a3, b3, c3, d3, col3, depthTest);
        }

        debug->AddLine(GetOwnWorldFrame().Translation(), GetOwnWorldFrame() * Vector3(commandedTorque_, 0, 0), Color::YELLOW, depthTest);
        debug->AddLine(GetOwnWorldFrame().Translation(), GetOwnWorldFrame() * Vector3(GetHingeAngularVelocity(), 0, 0), Color::RED, depthTest);


    }

    void NewtonRevoluteJoint::buildConstraint()
    {
        newtonConstraint_ = new ndRevoluteJoint(
            GetOwnNewtonBodyBuild()->GetAsBodyKinematic(), 
            GetOtherNewtonBodyBuild()->GetAsBodyKinematic(), 
            UrhoToNewton(GetOwnBuildWorldFrame()),
            UrhoToNewton(GetOtherBuildWorldFrame())
            );
    }

    bool NewtonRevoluteJoint::applyAllJointParams()
    {
        if (!NewtonConstraint::applyAllJointParams())
            return false;


        static_cast<ndRevoluteJoint*>(newtonConstraint_)->m_maxLimit = maxAngle_ * ndDegreeToRad;
        static_cast<ndRevoluteJoint*>(newtonConstraint_)->m_minLimit = minAngle_ * ndDegreeToRad;
        static_cast<ndRevoluteJoint*>(newtonConstraint_)->m_hasRotationLimits = enableHingeLimits_;
        static_cast<ndRevoluteJoint*>(newtonConstraint_)->m_internalFrictionCoef = Abs(frictionCoef_);
        static_cast<ndRevoluteJoint*>(newtonConstraint_)->SetTorque(-commandedTorque_);
    

        return true;
    }


    ndRevoluteJoint::ndRevoluteJoint(ndBodyKinematic* const body0, ndBodyKinematic* const body1, const ndMatrix& globalMatrix0, const ndMatrix& globalMatrix1) :
	ndJointBilateralConstraint(7, body0, body1, globalMatrix0, globalMatrix1),
    m_commandedTorque(0.0f),
	m_minLimit(-1.0f),
    m_maxLimit(1.0f),
    m_angle(0.0f),
	m_hasRotationLimits(false),
    m_hasOffsetLimits(true),
	m_internalFrictionCoef(0.0f)
    {
    }


    void ndRevoluteJoint::SetTorque(ndFloat32 newtonMeters)
    {
        m_commandedTorque = newtonMeters;
    }

    ndFloat32 ndRevoluteJoint::FinalTorque(ndConstraintDescritor& desc)
    {
        ndFloat32 frictionTorqueTerm = m_internalFrictionCoef * m_omega;
        ndFloat32 torque = m_commandedTorque - frictionTorqueTerm;
        return torque;
    }

    ndFloat32 ndRevoluteJoint::CalculateAcceleration(ndConstraintDescritor& desc, float resolvedTorque)
    {
        //"Lead" the angular velocity so the torque always the limiting factor.
        ndFloat32 diff = m_omega + 1e3f*Sign(resolvedTorque);
        ndFloat32 accel = diff * desc.m_invTimestep;
        return accel;
    }



    void ndRevoluteJoint::JacobianDerivative(ndConstraintDescritor& desc)
    {
        ndMatrix matrix0;
        ndMatrix matrix1;
        CalculateGlobalMatrix(matrix0, matrix1);

        const ndFloat32 angle0 = CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_up);
        const ndFloat32 angle1 = CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_right);

        ndVector omega0(m_body0->GetOmega());
        ndVector omega1(m_body1->GetOmega());

        const ndVector relOmega(omega0 - omega1);
        m_omega = matrix0.UnrotateVector(relOmega).GetX();


    	// the joint angle can be determined by getting the angle between any two non parallel vectors
        const ndFloat32 deltaAngle = AnglesAdd(-CalculateAngle(matrix0.m_up, matrix1.m_up, matrix1.m_front), -m_angle);
        m_angle += deltaAngle;

        if (m_hasOffsetLimits) {
            AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, matrix1.m_front);
            AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, matrix1.m_up);
            AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, matrix1.m_right);
        }

        AddAngularRowJacobian(desc, matrix1.m_up, angle0);
        AddAngularRowJacobian(desc, matrix1.m_right, angle1);


        float torque = FinalTorque(desc);


        if (m_hasRotationLimits)
        {
            if ((m_minLimit > ndFloat32(-1.e-4f)) && (m_maxLimit < ndFloat32(1.e-4f)))
            {
                AddAngularRowJacobian(desc, matrix1.m_front, -m_angle);
            }
            else
            {

                if (m_angle < m_minLimit)
                {
                    AddAngularRowJacobian(desc, matrix0.m_front, ndFloat32(0.0f));
                    const ndFloat32 stopAccel = GetMotorZeroAcceleration(desc);
                    const ndFloat32 penetration = m_angle - m_minLimit;
                    const ndFloat32 recoveringAceel = -desc.m_invTimestep * D_HINGE_PENETRATION_RECOVERY_SPEED * dMin(dAbs(penetration / D_HINGE_PENETRATION_LIMIT), ndFloat32(1.0f));
                    SetMotorAcceleration(desc, stopAccel - recoveringAceel);
                    SetLowerFriction(desc, 0);
                }
                else if (m_angle > m_maxLimit)
                {
                    AddAngularRowJacobian(desc, matrix0.m_front, ndFloat32(0.0f));
                    const ndFloat32 stopAccel = GetMotorZeroAcceleration(desc);
                    const ndFloat32 penetration = m_angle - m_maxLimit;
                    const ndFloat32 recoveringAceel = desc.m_invTimestep * D_HINGE_PENETRATION_RECOVERY_SPEED * dMin(dAbs(penetration / D_HINGE_PENETRATION_LIMIT), ndFloat32(1.0f));
                    SetMotorAcceleration(desc, stopAccel - recoveringAceel);
                    SetHighFriction(desc, 0);
                }
                else 
                {
                    AddAngularRowJacobian(desc, matrix0.m_front, ndFloat32(0.0f));
                    SetMotorAcceleration(desc, CalculateAcceleration(desc, torque));
                   
                    if (torque > 0.0f)
                    {
                        SetHighFriction(desc, torque);
                        SetLowerFriction(desc, -torque);
                    }
                    else
                    {
                        SetHighFriction(desc, -torque);
                        SetLowerFriction(desc, torque);
                    }
                }
            }
        }
        else
        {
            AddAngularRowJacobian(desc, matrix0.m_front, 0.0f);
            SetMotorAcceleration(desc, CalculateAcceleration(desc, torque));

            if(torque > 0.0f)
            {
                SetHighFriction(desc, torque);
                SetLowerFriction(desc, -torque);
            }
            else
            {
                SetHighFriction(desc, -torque);
                SetLowerFriction(desc, torque);
            }
        }
    }
}
