#pragma once

#include "ndPolygonSoupBuilder.h"
#include "UrhoNewtonApi.h"
#include "Urho3D/Core/Object.h"

class ndMeshEffect;

namespace Urho3D {


    class URHONEWTON_API NewtonMeshObject : public Object {

        URHO3D_OBJECT(NewtonMeshObject, Object);
    public:
        /// Construct.
        NewtonMeshObject(Context* context);
        /// Destruct. Free the rigid body and geometries.
        ~NewtonMeshObject() override;

        static void RegisterObject(Context* context);
        ndMeshEffect* mesh = nullptr;
        ndPolygonSoupBuilder newtonSoup;
    };


}
