#include "NewtonPhysicsWorld.h"
#include "NewtonSliderConstraint.h"
#include "UrhoNewtonConversions.h"
#include "NewtonRigidBody.h"

#include "Urho3D/IO/Log.h"
#include "Urho3D/Core/Context.h"


#define D_MAX_SLIDER_RECOVERY_SPEED	ndFloat32 (0.5f)
#define D_MAX_SLIDER_PENETRATION	ndFloat32 (0.05f)
namespace Urho3D
{
    NewtonSliderConstraint::NewtonSliderConstraint(Context* context) : NewtonConstraint(context)
    {

    }

    NewtonSliderConstraint::~NewtonSliderConstraint()
    {

    }

    void NewtonSliderConstraint::RegisterObject(Context* context)
    {
        context->RegisterFactory<NewtonSliderConstraint>(DEF_PHYSICS_CATEGORY.c_str());

        URHO3D_COPY_BASE_ATTRIBUTES(NewtonConstraint);
    	URHO3D_ACCESSOR_ATTRIBUTE("Slider Upper Limit", GetSliderUpperLimit, SetSliderUpperLimit, float, 0.0f, AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("Slider Lower Limit", GetSliderLowerLimit, SetSliderLowerLimit, float, 0.0f, AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("Slider Friction",    GetSliderFriction, SetSliderFriction,     float, 0.0f, AM_DEFAULT);
    }

    
    float NewtonSliderConstraint::GetDisplacement() const
    {
        if (newtonConstraint_)
        {
            return static_cast<SliderJoint*>(newtonConstraint_)->m_dist;
        }
        else
        {
            return 0.0f;
        }
    }

    float NewtonSliderConstraint::GetVel() const
    {
        if (newtonConstraint_)
        {
            return static_cast<SliderJoint*>(newtonConstraint_)->m_vel;
        }
        else
        {
            return 0.0f;
        }
    }

    void NewtonSliderConstraint::SetSliderLimits(float lowerLimit, float upperLimit)
    {
        SetSliderLowerLimit(lowerLimit);
        SetSliderUpperLimit(upperLimit);
    }

    void NewtonSliderConstraint::SetSliderUpperLimit(float upperLimit)
    {
        if (sliderLimits_.y_ != upperLimit)
        {
            sliderLimits_.y_ = upperLimit;
            if (newtonConstraint_)
            {
                static_cast<SliderJoint*>(newtonConstraint_)->m_maxLimit = sliderLimits_.y_;
            }
            else
                MarkDirty();
        }
    }

    void NewtonSliderConstraint::SetSliderLowerLimit(float lowerLimit)
    {
        if (sliderLimits_.x_ != lowerLimit)
        {
            sliderLimits_.x_ = lowerLimit;
            if (newtonConstraint_)
            {
                static_cast<SliderJoint*>(newtonConstraint_)->m_minLimit = sliderLimits_.x_;
            }
            else
                MarkDirty();
        }
    }

    void NewtonSliderConstraint::SetEnableSpin(bool enable)
    {
        if (enableSpin_ != enable)
        {
            enableSpin_ = enable;
            if (newtonConstraint_)
            {
                static_cast<SliderJoint*>(newtonConstraint_)->m_enableSpin = enable;
            }
            else
                MarkDirty();
        }
    }


    void NewtonSliderConstraint::SetSliderFriction(float friction)
    {
        if (frictionCoef_ != friction) {
            frictionCoef_ = friction;

            if (newtonConstraint_)
            {
                static_cast<SliderJoint*>(newtonConstraint_)->m_internalFrictionCoef = Abs(frictionCoef_);
            }
            else
                MarkDirty();
        }
    }



	void Urho3D::NewtonSliderConstraint::DrawDebugGeometry(DebugRenderer* debug, bool depthTest)
    {
        NewtonConstraint::DrawDebugGeometry(debug, depthTest);


        float limitSize = sliderLimits_.y_ - sliderLimits_.x_;
        limitSize = Clamp(limitSize, 0.1f, 0.5f);


        //draw limits
        if (limitSize < M_LARGE_VALUE) 
        {
            debug->AddLine(GetOwnWorldFrame() * Vector3(sliderLimits_.x_, 0, 0), GetOwnWorldFrame().Translation(), Color::RED, depthTest);
            debug->AddLine(GetOwnWorldFrame().Translation(), GetOwnWorldFrame() * Vector3(sliderLimits_.y_, 0, 0), Color::BLUE, depthTest);

            Vector3 a = GetOwnWorldFrame() * (Vector3(0, 1, -1) * limitSize + Vector3(sliderLimits_.x_, 0, 0));
            Vector3 b = GetOwnWorldFrame() * (Vector3(0, 1, 1) * limitSize + Vector3(sliderLimits_.x_, 0, 0));
            Vector3 c = GetOwnWorldFrame() * (Vector3(0, -1, 1) * limitSize + Vector3(sliderLimits_.x_, 0, 0));
            Vector3 d = GetOwnWorldFrame() * (Vector3(0, -1, -1) * limitSize + Vector3(sliderLimits_.x_, 0, 0));

            Vector3 a2 = GetOwnWorldFrame() * (Vector3(0, 1, -1) * limitSize + Vector3(sliderLimits_.y_, 0, 0));
            Vector3 b2 = GetOwnWorldFrame() * (Vector3(0, 1, 1) * limitSize + Vector3(sliderLimits_.y_, 0, 0));
            Vector3 c2 = GetOwnWorldFrame() * (Vector3(0, -1, 1) * limitSize + Vector3(sliderLimits_.y_, 0, 0));
            Vector3 d2 = GetOwnWorldFrame() * (Vector3(0, -1, -1) * limitSize + Vector3(sliderLimits_.y_, 0, 0));


            Color col1 = Color::RED;
            col1.a_ = 0.5f;
            Color col2 = Color::BLUE;
            col2.a_ = 0.5f;
            debug->AddPolygon(a, b, c, d, col1, depthTest);
            debug->AddPolygon(a2, b2, c2, d2, col2, depthTest);
        }

    }

	void NewtonSliderConstraint::SetCommandedForce(float force)
	{
        if (commandedForce_ != force) {
            commandedForce_ = force;

            if (newtonConstraint_)
            {
                static_cast<SliderJoint*>(newtonConstraint_)->m_commandedForce = commandedForce_;
            }
            else
                MarkDirty();
        }
	}

	void Urho3D::NewtonSliderConstraint::buildConstraint()
    {
        newtonConstraint_ = new SliderJoint(GetOwnNewtonBodyBuild()->GetAsBodyKinematic(),
            GetOtherNewtonBodyBuild()->GetAsBodyKinematic(), 
            UrhoToNewton(GetOwnBuildWorldFrame()),
            UrhoToNewton(GetOtherBuildWorldFrame())
			);
    }

    bool Urho3D::NewtonSliderConstraint::applyAllJointParams()
    {
        if (!NewtonConstraint::applyAllJointParams())
            return false;

        static_cast<SliderJoint*>(newtonConstraint_)->m_minLimit = sliderLimits_.x_;
        static_cast<SliderJoint*>(newtonConstraint_)->m_maxLimit = sliderLimits_.y_;
        static_cast<SliderJoint*>(newtonConstraint_)->m_internalFrictionCoef = Abs(frictionCoef_);
        static_cast<SliderJoint*>(newtonConstraint_)->m_commandedForce = commandedForce_;
        static_cast<SliderJoint*>(newtonConstraint_)->m_enableSpin = enableSpin_;

        return true;
    }


    SliderJoint::SliderJoint(ndBodyKinematic* const body0, 
        ndBodyKinematic* const body1,
        const ndMatrix& globalMatrix0,
        const ndMatrix& globalMatrix1)
        : ndJointBilateralConstraint(6, body0, body1, globalMatrix0, globalMatrix1)
        , m_dist(ndFloat32(0.0f))
        , m_vel(ndFloat32(0.0f))
        , m_minLimit(-FLT_MAX)
        , m_maxLimit(FLT_MAX)
		, m_commandedForce(0.0f)
        , m_internalFrictionCoef(0.0f)
		, m_enableSpin(false)
    {
    }



    ndFloat32 SliderJoint::CalculateAcceleration(ndConstraintDescritor& desc, float finalForce)
    {
        //"Lead" the angular velocity so the force is always the limiting factor
        ndFloat32 diff = m_vel + 1e3f * Sign(finalForce);
        ndFloat32 accel = diff * desc.m_invTimestep;
        return accel;
    }

    ndFloat32 SliderJoint::FinalForce(ndConstraintDescritor& desc)
    {
        ndFloat32 frictionTorqueTerm = m_internalFrictionCoef * m_vel;
        ndFloat32 force = m_commandedForce - frictionTorqueTerm;
        return force;
    }

    void SliderJoint::JacobianDerivative(ndConstraintDescritor& desc)
    {
        ndMatrix matrix0;
        ndMatrix matrix1;

        // calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
        CalculateGlobalMatrix(matrix0, matrix1);

        // calculate position and speed	
        const ndVector veloc0(m_body0->GetVelocityAtPoint(matrix0.m_posit));
        const ndVector veloc1(m_body1->GetVelocityAtPoint(matrix1.m_posit));

        const ndVector& pin = matrix1[0];
        const ndVector& p0 = matrix0.m_posit;
        const ndVector& p1 = matrix1.m_posit;
        const ndVector prel(p0 - p1);
        const ndVector vrel(veloc0 - veloc1);

        m_vel = vrel.DotProduct(matrix1.m_front).GetScalar();
        m_dist = prel.DotProduct(matrix1.m_front).GetScalar();
        const ndVector projectedPoint = p1 + pin.Scale(pin.DotProduct(prel).GetScalar());

        //Limits on axis other than the sliding
        AddLinearRowJacobian(desc, p0, projectedPoint, matrix1[1]);
        AddLinearRowJacobian(desc, p0, projectedPoint, matrix1[2]);

        const ndFloat32 angle0 = CalculateAngle(matrix0.m_up, matrix1.m_up, matrix1.m_front);
        AddAngularRowJacobian(desc, matrix1.m_front, angle0);

        const ndFloat32 angle1 = CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_up);
        AddAngularRowJacobian(desc, matrix1.m_up, angle1);

        const ndFloat32 angle2 = CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_right);
        AddAngularRowJacobian(desc, matrix1.m_right, angle2);

        float force = FinalForce(desc);


        ndFloat32 x = m_dist + m_vel * desc.m_timestep;
        if (x < m_minLimit)
        {
            AddLinearRowJacobian(desc, matrix0.m_posit, matrix0.m_posit, matrix1.m_front);
            const ndFloat32 stopAccel = GetMotorZeroAcceleration(desc);
            const ndFloat32 penetration = x - m_minLimit;
            const ndFloat32 recoveringAceel = -desc.m_invTimestep * D_MAX_SLIDER_RECOVERY_SPEED
        	* dMin(dAbs(penetration / D_MAX_SLIDER_PENETRATION), ndFloat32(1.0f));
            SetMotorAcceleration(desc, stopAccel - recoveringAceel);
            SetLowerFriction(desc, 0);
        }
        else if (x > m_maxLimit)
        {
            AddLinearRowJacobian(desc, matrix0.m_posit, matrix0.m_posit, matrix1.m_front);
            const ndFloat32 stopAccel = GetMotorZeroAcceleration(desc);
            const ndFloat32 penetration = x - m_maxLimit;
            const ndFloat32 recoveringAceel = desc.m_invTimestep * D_MAX_SLIDER_RECOVERY_SPEED
        	* dMin(dAbs(penetration / D_MAX_SLIDER_PENETRATION), ndFloat32(1.0f));
            SetMotorAcceleration(desc, stopAccel - recoveringAceel);
            SetHighFriction(desc, 0);
        }
        else
        {
            AddLinearRowJacobian(desc, matrix0.m_posit, matrix0.m_posit, matrix1.m_front);

            SetMotorAcceleration(desc, CalculateAcceleration(desc, force));
            if (force > 0.0f)
            {
                SetHighFriction(desc, force);
                SetLowerFriction(desc, -force);
            }
            else
            {
                SetHighFriction(desc, -force);
                SetLowerFriction(desc, force);
            }
        }
    }
}



