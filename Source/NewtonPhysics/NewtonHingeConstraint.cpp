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


#include "ndNewton.h"



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
        URHO3D_ACCESSOR_ATTRIBUTE("Torque", GetTorque, SetTorque, float, 0.0f, AM_DEFAULT);
    }

    void NewtonHingeConstraint::SetMinAngle(float minAngle)
    {
        if (minAngle_ != minAngle) {
            minAngle_ = minAngle;

            WakeBodies();

			if (newtonConstraint_)
			{
			//	static_cast<ndJointHinge*>(newtonConstraint_)->EnableLimits(enableLimits_, minAngle_ * ndDegreeToRad, maxAngle_ * ndDegreeToRad);
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
            if (newtonConstraint_) {
			//	static_cast<ndJointHinge*>(newtonConstraint_)->EnableLimits(enableLimits_, minAngle_ * ndDegreeToRad, maxAngle_ * ndDegreeToRad);
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
            if (newtonConstraint_) {
				//static_cast<ndJointHinge*>(newtonConstraint_)->EnableLimits(enableLimits_, minAngle_ * ndDegreeToRad, maxAngle_ * ndDegreeToRad);
            }
            else
                MarkDirty();
        }
    }

   

    void NewtonHingeConstraint::SetTorque(float torque)
    {
        if (commandedTorque_ != torque)
        {
            commandedTorque_ = torque;
            WakeBodies();
            if (newtonConstraint_)
            {
            	static_cast<PivotJoint*>(newtonConstraint_)->SetTorque(commandedTorque_);
            }
            else
                MarkDirty();
        }
    }

    float NewtonHingeConstraint::GetCurrentAngle()
    {
        if (newtonConstraint_)
        {
        	return static_cast<PivotJoint*>(newtonConstraint_)->GetAngle();
        }
        return 0.0f;
    }

    void NewtonHingeConstraint::DrawDebugGeometry(DebugRenderer* debug, bool depthTest)
    {
        NewtonConstraint::DrawDebugGeometry(debug, depthTest);
    }

    void NewtonHingeConstraint::buildConstraint()
    {
        newtonConstraint_ = new PivotJoint(
            GetOwnNewtonBodyBuild()->GetAsBodyKinematic(), 
            GetOtherNewtonBodyBuild()->GetAsBodyKinematic(), 
            UrhoToNewton(GetOwnBuildWorldFrame())
            );
    }

    bool NewtonHingeConstraint::applyAllJointParams()
    {
        if (!NewtonConstraint::applyAllJointParams())
            return false;



        //static_cast<PivotJoint*>(newtonConstraint_)->EnableLimits(enableLimits_, minAngle_ * ndDegreeToRad, maxAngle_ * ndDegreeToRad);
        static_cast<PivotJoint*>(newtonConstraint_)->SetTorque(commandedTorque_);
    

        return true;
    }


    PivotJoint::PivotJoint(ndBodyKinematic* const body0, ndBodyKinematic* const body1, const ndMatrix& globalMatrix) :
	ndJointBilateralConstraint(7, body0, body1, globalMatrix ),
    m_commandedTorque(0.0f),
	m_minLimit(-1.0f),
    m_maxLimit(1.0f),
	m_friction(0.0f),
	m_omega(0.0f)
    {

    }

    

    void PivotJoint::SetTorque(ndFloat32 newtonMeters)
    {
        m_commandedTorque = newtonMeters;
    }

    ndFloat32 PivotJoint::CalculateAcceleration(ndConstraintDescritor& desc)
    {
        const ndVector& relOmega = m_body1->GetOmega() + m_body0->GetOmega();

        ndFloat32 currentOmega = relOmega.AddHorizontal().GetScalar();
        ndFloat32 diff = Sign(m_commandedTorque)*1e9f -  currentOmega;

        ndFloat32 accel = diff * desc.m_invTimestep;
        return accel;
    }


    void PivotJoint::JacobianDerivative(ndConstraintDescritor& desc)
    {
        ndMatrix matrix0;
        ndMatrix matrix1;
        CalculateGlobalMatrix(matrix0, matrix1);

        const ndFloat32 angle0 = CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_up);
        const ndFloat32 angle1 = CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_right);

        ndVector omega0(m_body0->GetOmega());
        ndVector omega1(m_body1->GetOmega());

    	// the joint angle can be determined by getting the angle between any two non parallel vectors
        const ndFloat32 deltaAngle = AnglesAdd(-CalculateAngle(matrix0.m_up, matrix1.m_up, matrix1.m_front), -m_angle);
        m_angle += deltaAngle;

        AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, matrix1.m_front);
        AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, matrix1.m_up);
        AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, matrix1.m_right);


        AddAngularRowJacobian(desc, matrix1.m_up, angle0);
        AddAngularRowJacobian(desc, matrix1.m_right, angle1);
		AddAngularRowJacobian(desc, matrix0.m_front, 0.0f);

    	float acc = CalculateAcceleration(desc);
        float highFric = m_commandedTorque;
        float lowFric = -m_commandedTorque;




        SetMotorAcceleration(desc, acc);

        SetHighFriction(desc, highFric);
        SetLowerFriction(desc, lowFric);
        SetDiagonalRegularizer(desc, ndFloat32(0.1f));
    }
}
