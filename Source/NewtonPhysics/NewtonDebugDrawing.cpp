#include "NewtonPhysicsWorld.h"
#include "UrhoNewtonConversions.h"
#include "NewtonCollisionShape.h"
#include "NewtonRigidBody.h"
#include "NewtonDebugDrawing.h"


#include "Urho3D/Graphics/DebugRenderer.h"
#include "Urho3D/Math/Vector3.h"
#include "Urho3D/Scene/Component.h"
#include "Urho3D/Scene/Scene.h"


#include "ndNewton.h"


namespace Urho3D {






    //void NewtonDebug_ShowGeometryCollisionCallback(void* userData, int vertexCount, const ndFloat32* const faceVertec, int id)
    //{
    //    debugRenderOptions* options = static_cast<debugRenderOptions*>(userData);


    //    //if (mode == m_lines) {
    //    int index = vertexCount - 1;
    //    dVector p0(faceVertec[index * 3 + 0], faceVertec[index * 3 + 1], faceVertec[index * 3 + 2]);
    //    for (int i = 0; i < vertexCount; i++) {
    //        dVector p1(faceVertec[i * 3 + 0], faceVertec[i * 3 + 1], faceVertec[i * 3 + 2]);
    //        options->debug->AddLine(Vector3((p0.m_x), (p0.m_y), (p0.m_z)), Vector3((p1.m_x), (p1.m_y), (p1.m_z)), options->color, options->depthTest);

    //        p0 = p1;
    //    }

    //}

    



    void NewtonShapeDebugNotify::DrawPolygon(ndInt32 vertexCount, const ndVector* const faceArray,
	    const ndEdgeType* const edgeType)
    {
	    for(int v = 0; v < vertexCount; v++)
	    {
			if(v > 0)
				options.debug->AddLine(NewtonToUrhoVec3(faceArray[v - 1]), NewtonToUrhoVec3(faceArray[v]), Color::RED, options.depthTest);
	    }
    }

    NewtonConstraintDebugCallback::NewtonConstraintDebugCallback(DebugRenderer* debugRenderer, bool depthTest): ndConstraintDebugCallback()
    {
	    debugRenderer_ = debugRenderer;
	    depthTest_ = depthTest;
	    SetScale(0.5f);
    }

    NewtonConstraintDebugCallback::~NewtonConstraintDebugCallback()
    {}

    void NewtonConstraintDebugCallback::SetDrawScale(float scale)
    { worldScale_ = scale; }

    void NewtonConstraintDebugCallback::SetColor(const ndVector& color)
    {
        currentColor_ = Color(color.m_x, color.m_y, color.m_z);
    }

    void NewtonConstraintDebugCallback::DrawLine(const ndVector& p0, const ndVector& p1, const ndVector& color, ndFloat32 thickness )
    {
        debugRenderer_->AddLine(NewtonToUrhoVec3(p0)*worldScale_, NewtonToUrhoVec3(p1)*worldScale_, Color(color[0], color[1], color[2], color[3]), depthTest_);
    }



}
