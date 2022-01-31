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
        void SetTorque(float torque);
        float GetTorque()const { return commandedTorque_; }



        float GetCurrentAngle();



        virtual void DrawDebugGeometry(DebugRenderer* debug, bool depthTest) override;

    protected:

        float frictionTorque_ = 0.0f;
        bool  enableLimits_ = true;
        float minAngle_ = -45.0f;
        float maxAngle_ = 45.0f;

        float commandedTorque_ = 0.0f;

        virtual void buildConstraint() override;

        bool applyAllJointParams();
    };




    class PivotJoint : public ndJointBilateralConstraint
    {
    public:
        D_CLASS_REFLECTION(PivotJoint);
        PivotJoint(ndBodyKinematic* const body0, ndBodyKinematic* const body1, const ndMatrix& globalMatrix);
    	void SetTorque(ndFloat32 newtonMeters);
        ndFloat32 GetAngle()
        {
            return m_angle;
        }
    private:
        void AlignMatrix();
        ndFloat32 CalculateAcceleration(ndConstraintDescritor& desc);

        void JacobianDerivative(ndConstraintDescritor& desc);
    protected:
        ndFloat32 m_commandedTorque;
        ndFloat32 m_minLimit;
        ndFloat32 m_maxLimit;
        ndFloat32 m_angle;
        ndFloat32 m_omega;
        ndFloat32 m_friction;
    };






}
