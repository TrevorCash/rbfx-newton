#include "NewtonMeshObject.h"

#include "Urho3D/Core/Context.h"

#include "ndNewton.h"

namespace Urho3D {

    NewtonMeshObject::NewtonMeshObject(Context* context) : Object(context)
    {
    }

    NewtonMeshObject::~NewtonMeshObject()
    {
        if (mesh != nullptr)
            delete mesh;
    }

    void NewtonMeshObject::RegisterObject(Context* context)
    {
        context->RegisterFactory<NewtonMeshObject>();
    }
}
