#pragma once


#include "NewtonConstraint.h"


namespace Urho3D {
    class Context;



    class URHONEWTON_API NewtonRevoluteJoint : public NewtonConstraint
    {
        URHO3D_OBJECT(NewtonRevoluteJoint, NewtonConstraint);

    public:

        NewtonRevoluteJoint(Context* context);
        ~NewtonRevoluteJoint();


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

        //Get Relative Angle
        float GetAngle();

        //Local Axis of rotation.
        static Vector3 LocalHingeAxis() { return {1, 0, 0}; }

        //Get relative angular velocity on the hinge axis.
        float GetHingeAngularVelocity() const;


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
    class URHONEWTON_API ndRevoluteJoint : public ndJointBilateralConstraint
    {
    public:
        D_CLASS_REFLECTION(ndRevoluteJoint);
        ndRevoluteJoint(ndBodyKinematic* const body0, ndBodyKinematic* const body1, const ndMatrix& globalMatrix0, const ndMatrix& globalMatrix1);
    	void SetTorque(ndFloat32 newtonMeters);

        ndFloat32 m_commandedTorque;
        ndFloat32 m_minLimit;
        ndFloat32 m_maxLimit;
        bool m_hasLimits;
        ndFloat32 m_angle;
        ndFloat32 m_omega;
        ndFloat32 m_internalFrictionCoef;

    private:
        ndFloat32 CalculateAcceleration(ndConstraintDescritor& desc, float resolvedTorque);
        ndFloat32 FinalTorque(ndConstraintDescritor& desc);
        void JacobianDerivative(ndConstraintDescritor& desc);

    };


}
