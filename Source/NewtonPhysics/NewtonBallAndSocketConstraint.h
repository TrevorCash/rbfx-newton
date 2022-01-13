#pragma once

#include "UrhoNewtonApi.h"
#include "NewtonConstraint.h"


namespace Urho3D {
    class Context;

    

    class URHONEWTON_API NewtonBallAndSocketConstraint : public NewtonConstraint
    {
        URHO3D_OBJECT(NewtonBallAndSocketConstraint, NewtonConstraint);

    public:

        NewtonBallAndSocketConstraint(Context* context);
        ~NewtonBallAndSocketConstraint();

        static void RegisterObject(Context* context);


        ///Set Twist limits
        void SetTwistLimits(float minAngle, float maxAngle);
        void SetTwistLimitMin(float minAngle);
        void SetTwistLimitMax(float maxAngle);

        float GetTwistLimitMin() const;
        float GetTwistLimitMax() const;
        Vector2 GetTwistLimits() const;

        void SetTwistFriction(float frictionTorque);
        float GetTwistFriction() const;


        void SetConeAngle(float angle);
        float GetConeAngle() const;

        void SetConeFriction(float frictionTorque);
        float GetConeFriction() const;





        virtual void DrawDebugGeometry(DebugRenderer* debug, bool depthTest) override;

    protected:
        bool coneEnabled_ = true;
        bool twistLimitsEnabled_ = false;
        float coneAngle_ = 20.0f;
        float twistMinAngle_ = -45.0f;
        float twistMaxAngle_ = 45.0f;
        float twistFriction_ = 0.0f;
        float coneFriction_ = 0.0f;

        virtual void buildConstraint() override;

        virtual bool applyAllJointParams() override;

    };


}
