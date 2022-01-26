#pragma once
#include "ndNewton.h"
#include "Urho3D/Graphics/DebugRenderer.h"
#include "Urho3D/Math/Color.h"


namespace Urho3D
{
    class DebugRenderer;
    class Matrix3x4;
    class NewtonPhysicsWorld;




	class NewtonShapeDebugNotify : public ndShapeDebugNotify
	{
		public:
		virtual void DrawPolygon(ndInt32 vertexCount, const ndVector* const faceArray, const ndEdgeType* const edgeType);
		DebugRenderer* debugRenderer_ = nullptr;
	};




    class NewtonConstraintDebugCallback : public ndConstraintDebugCallback
    {
    public:
        NewtonConstraintDebugCallback(DebugRenderer* debugRenderer, bool depthTest);
        virtual ~NewtonConstraintDebugCallback();

        void SetDrawScale(float scale);

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





}
