#pragma once
#include "UrhoNewtonApi.h"
#include "NewtonConstraint.h"

namespace Urho3D {



    class Context;


    class URHONEWTON_API NewtonSliderConstraint : public NewtonConstraint
    {
        URHO3D_OBJECT(NewtonSliderConstraint, NewtonConstraint);

    public:

        NewtonSliderConstraint(Context* context);
        ~NewtonSliderConstraint();


        static void RegisterObject(Context* context);


        virtual void DrawDebugGeometry(DebugRenderer* debug, bool depthTest) override;

        void SetCommandedForce(float force);

        //Get the relative displacement of the slider
        float GetDisplacement() const;
        float GetVel() const;

        ///Set the distance limits the bodies with be able to slide. lower limit should be negative
        void SetSliderLimits(float lowerLimit, float upperLimit);
        void SetSliderUpperLimit(float upperLimit);
        float GetSliderUpperLimit() const { return sliderLimits_.y_; }
        void SetSliderLowerLimit(float lowerLimit);
        float GetSliderLowerLimit() const { return sliderLimits_.x_; }

        void SetEnableSpin(bool enable);

        ///Set the sliding friction coefficient
        void SetSliderFriction(float friction);
        float GetSliderFriction() const { return frictionCoef_; }


    protected:

        float commandedForce_ = 0.0f;
        bool enableLowerSliderLimit_ = false;
        bool enableUpperSliderLimit_ = false;
        Vector2 sliderLimits_ = Vector2(-FLT_MAX, FLT_MAX);
        bool enableSpin_ = false;

        float frictionCoef_ = 0.0f;

        virtual void buildConstraint() override;

        bool applyAllJointParams();
    };




    //A Simple slider Joint with commandable torque and internal friction coefficient with linear limits.
    class URHONEWTON_API SliderJoint : public ndJointBilateralConstraint
    {
    public:
        D_CLASS_REFLECTION(SliderJoint);
        SliderJoint(ndBodyKinematic* const body0, ndBodyKinematic* const body1, const ndMatrix& globalMatrix0, const ndMatrix& globalMatrix1);

        ndFloat32 m_commandedForce;
        ndFloat32 m_minLimit;
        ndFloat32 m_maxLimit;
        ndFloat32 m_internalFrictionCoef;
        ndFloat32 m_vel;
        ndFloat32 m_dist;
        bool m_enableSpin;
    private:
        ndFloat32 CalculateAcceleration(ndConstraintDescritor& desc, float resolvedTorque);
        ndFloat32 FinalForce(ndConstraintDescritor& desc);
        void JacobianDerivative(ndConstraintDescritor& desc);



    };








}
