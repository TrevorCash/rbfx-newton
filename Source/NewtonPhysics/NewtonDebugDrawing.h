#pragma once
#include "ndNewton.h"
#include "Urho3D/Graphics/DebugRenderer.h"
#include "Urho3D/Math/Color.h"


namespace Urho3D
{
    class DebugRenderer;
    class Matrix3x4;
    class NewtonPhysicsWorld;


    //class enabling native newton debug calls using Urho3D::DebugRenderer.
    class UrhoNewtonDebugDisplay : public ndConstraintDebugCallback
    {
    public:
        UrhoNewtonDebugDisplay(DebugRenderer* debugRenderer, bool depthTest) : ndConstraintDebugCallback()
        {
            debugRenderer_ = debugRenderer;
            depthTest_ = depthTest;
            SetScale(0.5f);
        }
        virtual ~UrhoNewtonDebugDisplay() {}

        void SetDrawScale(float scale) { worldScale_ = scale; }

        virtual void SetColor(const ndVector& color);
        virtual void DrawLine(const ndVector& p0, const ndVector& p1, const ndVector& color, ndFloat32 thickness = ndFloat32(1.0f));


	protected:
        float worldScale_ = 1.0f;
        Color currentColor_;
        bool depthTest_ = false;
        DebugRenderer* debugRenderer_ = nullptr;
    };




    struct debugRenderOptions {
        Color color = Color::GRAY;
        DebugRenderer* debug;
        bool depthTest = false;
    };



    //void NewtonDebug_BodyDrawCollision(NewtonPhysicsWorld* physicsWorld, const NewtonBody* const body, DebugRenderer* debug, bool depthTest = false);
    //void NewtonDebug_DrawCollision(NewtonCollision* collision, const Matrix3x4& transform, const Color& color, DebugRenderer* debug, bool depthTest = false);

    //void NewtonDebug_ShowGeometryCollisionCallback(void* userData, int vertexCount, const ndFloat32* const faceVertec, int id);



}
