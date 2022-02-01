#pragma once


#include "NewtonConstraint.h"


namespace Urho3D {
    class Context;


#define HINGE_CONSTRAINT_DEF_SPRING_COEF 100.0f
#define HINGE_CONSTRAINT_DEF_DAMPER_COEF 1.0f
#define HINGE_CONSTRAINT_DEF_RELAX 0.9f



    class URHONEWTON_API NewtonHingeConstraint : public NewtonConstraint
    {
        URHO3D_OBJECT(NewtonHingeConstraint, NewtonConstraint);

    public:

        NewtonHingeConstraint(Context* context);
        ~NewtonHingeConstraint();


        static void RegisterObject(Context* context);

        void SetMinAngle(float minAngle);
        float GetMinAngle() const { return minAngle_; }

        void SetMaxAngle(float maxAngle);
        float GetMaxAngle() const { return maxAngle_; }

        void SetEnableLimits(bool enable);
        bool GetLimitsEnabled() const { return enableLimits_; }

        /// Set max torque for actuator powered modes.
        void SetCommandedTorque(float torque);
        float GetCommandedTorque()const { return commandedTorque_; }

        void SetFrictionCoef(float frictionCoef);
        float GetFrictionCoef() const { return frictionCoef_; }

        float GetCurrentAngle();



        virtual void DrawDebugGeometry(DebugRenderer* debug, bool depthTest) override;

    protected:

        float frictionCoef_ = 0.1f;
        bool  enableLimits_ = true;
        float minAngle_ = -45.0f;
        float maxAngle_ = 45.0f;
        float commandedTorque_ = 0.0f;

        virtual void buildConstraint() override;

        bool applyAllJointParams();
    };


    //A Simple Hinge Joint with commandable torque and internal friction coefficient.
    //Option Angular Limits.
    class URHONEWTON_API PivotJoint : public ndJointBilateralConstraint
    {
    public:
        D_CLASS_REFLECTION(PivotJoint);
        PivotJoint(ndBodyKinematic* const body0, ndBodyKinematic* const body1, const ndMatrix& globalMatrix);
    	 void SetTorque(ndFloat32 newtonMeters);

        ndFloat32 GetAngle() const
        {
            return m_angle;
        }

        ndFloat32 m_commandedTorque;
        ndFloat32 m_minLimit;
        ndFloat32 m_maxLimit;
        bool m_hasLimits;
        ndFloat32 m_angle;
        ndFloat32 m_omega;
        ndFloat32 m_limitsFriction;
        ndFloat32 m_internalFriction;

    private:
        void AlignMatrix();
        ndFloat32 CalculateAcceleration(ndConstraintDescritor& desc, float resolvedTorque);
        ndFloat32 ResolvedTorque(ndConstraintDescritor& desc);
        void JacobianDerivative(ndConstraintDescritor& desc);

    };






}
