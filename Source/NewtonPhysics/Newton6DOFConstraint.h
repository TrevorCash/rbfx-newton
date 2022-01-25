#pragma once
//#include "UrhoNewtonApi.h"
//#include "NewtonConstraint.h"
//
//
//namespace Urho3D {
//    class Context;
//
//
//
//
//    class URHONEWTON_API NewtonSixDofConstraint : public NewtonConstraint
//    {
//        URHO3D_OBJECT(NewtonSixDofConstraint, NewtonConstraint);
//
//    public:
//
//        NewtonSixDofConstraint(Context* context);
//        ~NewtonSixDofConstraint();
//
//
//
//        static void RegisterObject(Context* context);
//
//
//        ////Set the relative space the joint can occupy.v
//        //void SetSpaceLimits();
//
//        void SetPitchLimits(float minLimit, float maxLimit);
//        void SetPitchLimits(const Vector3& limits);
//        
//        void SetYawLimits(float minLimit, float maxLimit);
//        void SetYawLimits(const Vector3& limits);
//
//        void SetRollLimits(float minLimit, float maxLimit);
//        void SetRollLimits(const Vector3& limits);
//
//
//       
//        virtual void DrawDebugGeometry(DebugRenderer* debug, bool depthTest) override;
//
//    protected:
//
//        Vector2 pitchLimits_;
//        Vector2 yawLimits_;
//        Vector2 rollLimits_;
//
//
//
//        virtual void buildConstraint() override;
//
//        bool applyAllJointParams();
//    };
//
//
//}
