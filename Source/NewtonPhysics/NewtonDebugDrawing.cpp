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






    void NewtonDebug_ShowGeometryCollisionCallback(void* userData, int vertexCount, const ndFloat* const faceVertec, int id)
    {
        debugRenderOptions* options = static_cast<debugRenderOptions*>(userData);


        //if (mode == m_lines) {
        int index = vertexCount - 1;
        dVector p0(faceVertec[index * 3 + 0], faceVertec[index * 3 + 1], faceVertec[index * 3 + 2]);
        for (int i = 0; i < vertexCount; i++) {
            dVector p1(faceVertec[i * 3 + 0], faceVertec[i * 3 + 1], faceVertec[i * 3 + 2]);
            options->debug->AddLine(Vector3((p0.m_x), (p0.m_y), (p0.m_z)), Vector3((p1.m_x), (p1.m_y), (p1.m_z)), options->color, options->depthTest);

            p0 = p1;
        }

    }

    void NewtonDebug_BodyDrawCollision(NewtonPhysicsWorld* physicsWorld, const NewtonBody* const body, DebugRenderer* debug, bool depthTest /*= false*/)
    {
        debugRenderOptions options;
        options.debug = debug;
        options.depthTest = depthTest;


        switch (NewtonBodyGetType(body))
        {
        case NEWTON_DYNAMIC_BODY:
        {
            int sleepState = NewtonBodyGetSleepState(body);
            if (sleepState == 1) {
                // indicate when body is sleeping
                options.color = Color::BLUE;
            }
            else {
                // body is active
                options.color = Color::RED;
            }
            break;
        }

        case NEWTON_KINEMATIC_BODY:
            options.color = Color::WHITE;
            break;
        }

        NewtonRigidBody* rigBodyComp = (NewtonRigidBody*)NewtonBodyGetUserData(body);
        if (!rigBodyComp)
            return;

        for (NewtonCollisionShape* colShapeComp : rigBodyComp->GetCollisionShapes())
        {

            if (!colShapeComp->GetDrawNewtonDebugGeometry())
                continue;


            dMatrix matrix;
            NewtonBodyGetMatrix(body, &matrix[0][0]);
            Matrix3x4 mat = Matrix3x4(NewtonToUrhoMat4(matrix));
            mat = colShapeComp->GetWorldTransform();

           
            matrix = UrhoToNewton(mat);
            NewtonCollisionForEachPolygonDo(colShapeComp->GetNewtonCollision(), &matrix[0][0], NewtonDebug_ShowGeometryCollisionCallback, (void*)&options);

        }
    }


    void NewtonDebug_DrawCollision(NewtonCollision* collision, const Matrix3x4& transform, const Color& color, DebugRenderer* debug, bool depthTest /*= false*/)
    {
        debugRenderOptions options;
        options.debug = debug;
        options.color = color;
        options.depthTest = depthTest;

        NewtonCollisionForEachPolygonDo(collision, &UrhoToNewton(transform)[0][0], NewtonDebug_ShowGeometryCollisionCallback, (void*)&options);
    }

    void UrhoNewtonDebugDisplay::SetColor(const dVector& color)
    {
        currentColor_ = Color(color.m_x, color.m_y, color.m_z);
    }

    void UrhoNewtonDebugDisplay::DrawLine(const dVector& p0, const dVector& p1)
    {
        debugRenderer_->AddLine(NewtonToUrhoVec3(p0)*worldScale_, NewtonToUrhoVec3(p1)*worldScale_, currentColor_, depthTest_);
    }

	void UrhoNewtonDebugDisplay::DrawPoint(const dVector& p0, dFloat thinckness /*= 1.0f*/)
	{
		throw std::logic_error("The method or operation is not implemented.");
	}

	void UrhoNewtonDebugDisplay::SetOrthRendering()
	{
		
	}

	void UrhoNewtonDebugDisplay::ResetOrthRendering()
	{
		
	}

}
