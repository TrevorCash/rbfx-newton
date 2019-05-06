#pragma once
#include "UrhoNewtonApi.h"
#include "Urho3D/Core/Object.h"
class NewtonMesh;
namespace Urho3D {


    //Reference countable newtonMesh 
    class URHONEWTON_API NewtonMeshObject : public Object {

        URHO3D_OBJECT(NewtonMeshObject, Object);
    public:
        /// Construct.
        NewtonMeshObject(Context* context);
        /// Destruct. Free the rigid body and geometries.
        ~NewtonMeshObject() override;

        static void RegisterObject(Context* context);
        NewtonMesh* mesh = nullptr;
    };


}
