#include "NewtonCollisionShapesDerived.h"
#include "NewtonPhysicsWorld.h"
#include "NewtonRigidBody.h"
#include "NewtonMeshObject.h"
#include "UrhoNewtonConversions.h"
#include "NewtonCollisionShape.h"


#include "Urho3D/Core/Context.h"
#include "Urho3D/Scene/Component.h"
#include "Urho3D/Scene/Node.h"
#include "Urho3D/Scene/Scene.h"
#include "Urho3D/Graphics/Model.h"
#include "Urho3D/Resource/Image.h"
#include "Urho3D/IO/Log.h"
#include "Urho3D/Graphics/Geometry.h"
#include "Urho3D/Graphics/VertexBuffer.h"
#include "Urho3D/Graphics/GraphicsDefs.h"
#include "Urho3D/Graphics/IndexBuffer.h"
#include "Urho3D/Graphics/StaticModel.h"
#include "Urho3D/Scene/SceneEvents.h"
#include "Urho3D/Graphics/Terrain.h"
#include "Urho3D/Resource/ResourceCache.h"
#include "Urho3D/Graphics/TerrainPatch.h"
#include "Urho3D/Math/StringHash.h"


#include "ndNewton.h"
#include "ndShapeBox.h"


namespace Urho3D {

    NewtonCollisionShape_Box::NewtonCollisionShape_Box(Context* context) : NewtonCollisionShape(context)
    {

    }

    NewtonCollisionShape_Box::~NewtonCollisionShape_Box()
    {

    }

    void NewtonCollisionShape_Box::RegisterObject(Context* context)
    {
        context->RegisterFactory<NewtonCollisionShape_Box>(DEF_PHYSICS_CATEGORY.c_str());
        URHO3D_COPY_BASE_ATTRIBUTES(NewtonCollisionShape);
        URHO3D_ACCESSOR_ATTRIBUTE("Size", GetSize, SetSize, Vector3, Vector3::ONE, AM_DEFAULT);

    }

    bool NewtonCollisionShape_Box::buildNewtonCollision()
    {
        //newtonShape_ = NewtonCreateBox(physicsWorld_->GetNewtonWorld(), size_.x_,
        //    size_.y_,
        //    size_.z_, 0, nullptr);
		newtonShape_ = ndShapeInstance(new ndShapeBox(static_cast<ndFloat32>(size_.x_), static_cast<ndFloat32>(size_.y_), static_cast<ndFloat32>(size_.z_)));


        return true;
    }







    NewtonCollisionShape_Sphere::NewtonCollisionShape_Sphere(Context* context) : NewtonCollisionShape(context)
    {

    }

    NewtonCollisionShape_Sphere::~NewtonCollisionShape_Sphere()
    {

    }

    void NewtonCollisionShape_Sphere::RegisterObject(Context* context)
    {
        context->RegisterFactory<NewtonCollisionShape_Sphere>(DEF_PHYSICS_CATEGORY.c_str());
        URHO3D_COPY_BASE_ATTRIBUTES(NewtonCollisionShape);
        URHO3D_ACCESSOR_ATTRIBUTE("Radius", GetRadius, SetRadius, float, 0.5f, AM_DEFAULT);
    }

    bool NewtonCollisionShape_Sphere::buildNewtonCollision()
{
        // get a newton collision object (note: the same NewtonCollision could be shared between multiple component so this is not nessecarily a unique pointer)
        //newtonShape_ = NewtonCreateSphere(physicsWorld_->GetNewtonWorld(), radius_, 0, nullptr);
		newtonShape_ = new ndShapeSphere(radius_);

        return true;
    }




    NewtonCollisionShape_Geometry::NewtonCollisionShape_Geometry(Context* context) : NewtonCollisionShape(context)
    {

    }

    NewtonCollisionShape_Geometry::~NewtonCollisionShape_Geometry()
    {

    }

    void NewtonCollisionShape_Geometry::RegisterObject(Context* context)
    {
        context->RegisterFactory<NewtonCollisionShape_Geometry>(DEF_PHYSICS_CATEGORY.c_str());
        URHO3D_COPY_BASE_ATTRIBUTES(NewtonCollisionShape);
        URHO3D_MIXED_ACCESSOR_ATTRIBUTE("Model", GetModelResourceRef, SetModelByResourceRef, ResourceRef, ResourceRef(Model::GetTypeStatic()), AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("Model Lod", GetModelLodLevel, SetModelLodLevel, int, 0, AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("Hull Tolerance", GetHullTolerance, SetHullTolerance, float, 0.0f, AM_DEFAULT);
        


    }

    void NewtonCollisionShape_Geometry::SetModelByResourceRef(const ResourceRef& ref)
    {
        auto* cache = GetSubsystem<ResourceCache>();
        SetModel(WeakPtr<Model>(cache->GetResource<Model>(ref.name_)));
    }

    bool NewtonCollisionShape_Geometry::resolveOrCreateTriangleMeshFromModel()
{
        if (!model_)
            return false;

        ndWorld* world = physicsWorld_->GetNewtonWorld();

        /// if the newton mesh is in cache already - return that.
        StringHash meshKey = NewtonPhysicsWorld::NewtonMeshKey(model_->GetName(), modelLodLevel_, "");
        NewtonMeshObject* cachedMesh = physicsWorld_->GetNewtonMesh(meshKey);
        if (cachedMesh)
        {
            newtonMesh_ = cachedMesh;
            return true;
        }

        Geometry* geo = model_->GetGeometry(modelGeomIndx_, modelLodLevel_);
        newtonMesh_ = getCreateTriangleMesh(geo, meshKey);
        if(!newtonMesh_)
        { 
            URHO3D_LOGWARNING("Unable To Create NewtonMesh For Model: " + model_->GetName());
            return false;
        }

        return true;
    }

    NewtonMeshObject* NewtonCollisionShape_Geometry::getCreateTriangleMesh(Geometry* geom, StringHash meshkey)
    {
        NewtonMeshObject* cachedMesh = physicsWorld_->GetNewtonMesh(meshkey);
        if (cachedMesh)
        {
            return cachedMesh;
        }


        const unsigned char* vertexData;
        const unsigned char* indexData;
        unsigned elementSize, indexSize;
		const  ea::vector<VertexElement>* elements;

        geom->GetRawData(vertexData, elementSize, indexData, indexSize, elements);

        bool hasPosition = VertexBuffer::HasElement(*elements, TYPE_VECTOR3, SEM_POSITION);

        if (vertexData && indexData && hasPosition)
        {
            unsigned vertexStart = geom->GetVertexStart();
            unsigned vertexCount = geom->GetVertexCount();
            unsigned indexStart = geom->GetIndexStart();
            unsigned indexCount = geom->GetIndexCount();

            unsigned positionOffset = VertexBuffer::GetElementOffset(*elements, TYPE_VECTOR3, SEM_POSITION);


            cachedMesh = physicsWorld_->GetCreateNewtonMesh(meshkey);
           
            cachedMesh->newtonSoup.Begin();


            for (unsigned curIdx = indexStart; curIdx < indexStart + indexCount; curIdx += 3)
            {
                //get indexes
                unsigned i1, i2, i3;
                if (indexSize == 2) {
                    i1 = *reinterpret_cast<const unsigned short*>(indexData + curIdx * indexSize);
                    i2 = *reinterpret_cast<const unsigned short*>(indexData + (curIdx + 1) * indexSize);
                    i3 = *reinterpret_cast<const unsigned short*>(indexData + (curIdx + 2) * indexSize);
                }
                else if (indexSize == 4)
                {
                    i1 = *reinterpret_cast<const unsigned *>(indexData + curIdx * indexSize);
                    i2 = *reinterpret_cast<const unsigned *>(indexData + (curIdx + 1) * indexSize);
                    i3 = *reinterpret_cast<const unsigned *>(indexData + (curIdx + 2) * indexSize);
                }

                //lookup triangle using indexes.
                const Vector3& v1 = *reinterpret_cast<const Vector3*>(vertexData + i1 * elementSize + positionOffset);
                const Vector3& v2 = *reinterpret_cast<const Vector3*>(vertexData + i2 * elementSize + positionOffset);
                const Vector3& v3 = *reinterpret_cast<const Vector3*>(vertexData + i3 * elementSize + positionOffset);

                //Todo optimize.
                ndFloat32 faceVerts[3 * 3];
                faceVerts[0] = v1.x_;
                faceVerts[1] = v1.y_;
                faceVerts[2] = v1.z_;
                faceVerts[3] = v2.x_;
                faceVerts[4] = v2.y_;
                faceVerts[5] = v2.z_;
                faceVerts[6] = v3.x_;
                faceVerts[7] = v3.y_;
                faceVerts[8] = v3.z_;


                //cachedMesh->mesh->BeginBuildFace();

                //cachedMesh->mesh->AddPoint(v1.x_, v1.y_, v1.z_);
                //cachedMesh->mesh->AddPoint(v2.x_, v2.y_, v2.z_);
                //cachedMesh->mesh->AddPoint(v3.x_, v3.y_, v3.z_);

                //cachedMesh->mesh->EndBuildFace();

                cachedMesh->newtonSoup.AddFace(faceVerts, sizeof(ndFloat32)*3, 3, curIdx);

            }

            cachedMesh->newtonSoup.End(true);
        }

        return cachedMesh;
    }

    void NewtonCollisionShape_Geometry::OnNodeSet(Node* node)
    {
        NewtonCollisionShape::OnNodeSet(node);

        if (node)
            autoSetModel();
    }

    bool NewtonCollisionShape_Geometry::buildNewtonCollision()
{
        return resolveOrCreateTriangleMeshFromModel();
    }


    void NewtonCollisionShape_Geometry::autoSetModel()
    {
        StaticModel* stMdl = node_->GetComponent<StaticModel>();

        if (stMdl)
        {
            SetModel(WeakPtr<Model>( stMdl->GetModel() ));
        }
    }

    Urho3D::ResourceRef NewtonCollisionShape_Geometry::GetModelResourceRef() const
    {
        return GetResourceRef(model_, Model::GetTypeStatic());
    }


//
//    NewtonCollisionShape_ConvexHullCompound::NewtonCollisionShape_ConvexHullCompound(Context* context) : NewtonCollisionShape_Geometry(context)
//    {
//    }
//
//    NewtonCollisionShape_ConvexHullCompound::~NewtonCollisionShape_ConvexHullCompound()
//    {
//    }
//
//    void NewtonCollisionShape_ConvexHullCompound::RegisterObject(Context* context)
//    {
//        context->RegisterFactory<NewtonCollisionShape_ConvexHullCompound>(DEF_PHYSICS_CATEGORY.c_str());
//        URHO3D_COPY_BASE_ATTRIBUTES(NewtonCollisionShape_Geometry);
//    }
//
//
//    bool NewtonCollisionShape_ConvexHullCompound::buildNewtonCollision()
//{
//        NewtonWorld* world = physicsWorld_->GetNewtonWorld();
//
//        if (!resolveOrCreateTriangleMeshFromModel())
//            return false;
//
//        newtonCollision_ = NewtonCreateCompoundCollisionFromMesh(world, newtonMesh_->mesh, hullTolerance_, 0, 0);
//
//        return true;
//    }




    //NewtonCollisionShape_ConvexDecompositionCompound::NewtonCollisionShape_ConvexDecompositionCompound(Context* context) : NewtonCollisionShape_Geometry(context)
    //{

    //}



    //NewtonCollisionShape_ConvexDecompositionCompound::~NewtonCollisionShape_ConvexDecompositionCompound()
    //{

    //}




    //void NewtonCollisionShape_ConvexDecompositionCompound::RegisterObject(Context* context)
    //{
    //    context->RegisterFactory<NewtonCollisionShape_ConvexDecompositionCompound>(DEF_PHYSICS_CATEGORY.c_str());
    //    URHO3D_COPY_BASE_ATTRIBUTES(NewtonCollisionShape_Geometry);
    //}

    //bool NewtonCollisionShape_ConvexDecompositionCompound::buildNewtonCollision()
    //{
    //    return NewtonCollisionShape_Geometry::buildNewtonCollision();

    //    //NewtonWorld* world = physicsWorld_->GetNewtonWorld();


    //    //String keyData = String(maxConcavity_) + String(backFaceDistanceFactor_) + String(maxCompounds_) + String(maxVertexPerHull_);

    //    ///// if the newton mesh is in cache already - return that.
    //    //StringHash meshKey = UrhoNewtonPhysicsWorld::NewtonMeshKey(model_->GetName(), modelLodLevel_, keyData);
    //    //NewtonMeshObject* cachedMesh = physicsWorld_->GetNewtonMesh(meshKey);
    //    //if (cachedMesh)
    //    //{
    //    //    meshDecomposition_ = cachedMesh;
    //    //}
    //    //else
    //    //{
    //    //    meshDecomposition_ = physicsWorld_->GetCreateNewtonMesh(meshKey);
    //    //    meshDecomposition_->mesh = NewtonMeshApproximateConvexDecomposition(newtonMesh_->mesh, maxConcavity_, backFaceDistanceFactor_, maxCompounds_, maxVertexPerHull_, nullptr, nullptr);

    //    //}

    //    //newtonCollision_ = NewtonCreateCompoundCollisionFromMesh(world, meshDecomposition_->mesh, hullTolerance_, 0, 0);
    //}

    //NewtonCollisionShape_ConvexHull::NewtonCollisionShape_ConvexHull(Context* context) : NewtonCollisionShape_Geometry(context)
    //{

    //}

    //NewtonCollisionShape_ConvexHull::~NewtonCollisionShape_ConvexHull()
    //{

    //}

    //void NewtonCollisionShape_ConvexHull::RegisterObject(Context* context)
    //{
    //    context->RegisterFactory<NewtonCollisionShape_ConvexHull>(DEF_PHYSICS_CATEGORY.c_str());
    //    URHO3D_COPY_BASE_ATTRIBUTES(NewtonCollisionShape_Geometry);
    //}

    //bool NewtonCollisionShape_ConvexHull::buildNewtonCollision()
    //{
    //    if (!NewtonCollisionShape_Geometry::buildNewtonCollision())
    //        return false;

    //    NewtonWorld* world = physicsWorld_->GetNewtonWorld();

    //    newtonCollision_ = NewtonCreateConvexHullFromMesh(world, newtonMesh_->mesh, hullTolerance_, 0);

    //    dMatrix offsetMat;
    //    NewtonCollisionGetMatrix(newtonCollision_, &offsetMat[0][0]);

    //    NewtonCollisionSetMatrix(newtonCollision_, &dGetIdentityMatrix()[0][0]);


    //    //offset by model geometry center (#todo this may be a quirk of newton - CompoundHulling does not need this.)
    //    position_ += NewtonToUrhoVec3(offsetMat.m_posit);//model_->GetGeometryCenter(modelLodLevel_);
    //    return true;
    //}






    NewtonCollisionShape_Cylinder::NewtonCollisionShape_Cylinder(Context* context) : NewtonCollisionShape(context)
    {
    }

    NewtonCollisionShape_Cylinder::~NewtonCollisionShape_Cylinder()
    {

    }

    void NewtonCollisionShape_Cylinder::RegisterObject(Context* context)
    {
        context->RegisterFactory<NewtonCollisionShape_Cylinder>(DEF_PHYSICS_CATEGORY.c_str());
        URHO3D_COPY_BASE_ATTRIBUTES(NewtonCollisionShape);
        URHO3D_ACCESSOR_ATTRIBUTE("Radius 1", GetRadius1, SetRadius1, float, 0.5f, AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("Radius 2", GetRadius2, SetRadius2, float, 0.5f, AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("Length", GetLength, SetLength, float, 1.0f, AM_DEFAULT);
    }

    bool NewtonCollisionShape_Cylinder::buildNewtonCollision()
    {
        // get a newton collision object (note: the same NewtonCollision could be shared between multiple component so this is not nessecarily a unique pointer)
        //newtonCollision_ = NewtonCreateCylinder(physicsWorld_->GetNewtonWorld(), radius1_, radius2_, length_, 0, nullptr);
		newtonShape_ = new ndShapeCylinder(radius1_, radius2_, length_);
        return true;
    }


    NewtonCollisionShape_ChamferCylinder::NewtonCollisionShape_ChamferCylinder(Context* context) : NewtonCollisionShape(context)
    {

    }

    NewtonCollisionShape_ChamferCylinder::~NewtonCollisionShape_ChamferCylinder()
    {

    }

    void NewtonCollisionShape_ChamferCylinder::RegisterObject(Context* context)
    {
        context->RegisterFactory<NewtonCollisionShape_ChamferCylinder>(DEF_PHYSICS_CATEGORY.c_str());
        URHO3D_COPY_BASE_ATTRIBUTES(NewtonCollisionShape);
        URHO3D_ACCESSOR_ATTRIBUTE("Radius", GetRadius, SetRadius, float, 0.5f, AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("Length", GetLength, SetLength, float, 1.0f, AM_DEFAULT);
    }

    bool NewtonCollisionShape_ChamferCylinder::buildNewtonCollision()
    {
        // get a newton collision object (note: the same NewtonCollision could be shared between multiple component so this is not nessecarily a unique pointer)
        //newtonShape_ = NewtonCreateChamferCylinder(physicsWorld_->GetNewtonWorld(), radius_, length_, 0, nullptr);
		newtonShape_ = new ndShapeChamferCylinder(radius_, length_);

        return true;
    }




    NewtonCollisionShape_Capsule::NewtonCollisionShape_Capsule(Context* context) : NewtonCollisionShape(context)
    {
    }

    NewtonCollisionShape_Capsule::~NewtonCollisionShape_Capsule()
    {
    }

    void NewtonCollisionShape_Capsule::RegisterObject(Context* context)
    {
        context->RegisterFactory<NewtonCollisionShape_Capsule>(DEF_PHYSICS_CATEGORY.c_str());
        URHO3D_COPY_BASE_ATTRIBUTES(NewtonCollisionShape);
        URHO3D_ACCESSOR_ATTRIBUTE("Radius 1", GetRadius1, SetRadius1, float, 0.5f, AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("Radius 2", GetRadius2, SetRadius2, float, 0.5f, AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("Length", GetLength, SetLength, float, 1.0f, AM_DEFAULT);
    }

    bool NewtonCollisionShape_Capsule::buildNewtonCollision()
{
        //newtonShape_ = NewtonCreateCapsule(physicsWorld_->GetNewtonWorld(), radius1_, radius2_, length_, 0, nullptr);
		newtonShape_ = new ndShapeCapsule(radius1_, radius2_, length_);
        return true;
    }

    NewtonCollisionShape_Cone::NewtonCollisionShape_Cone(Context* context) : NewtonCollisionShape(context)
    {
    }

    NewtonCollisionShape_Cone::~NewtonCollisionShape_Cone()
    {
    }

    void NewtonCollisionShape_Cone::RegisterObject(Context* context)
    {
        context->RegisterFactory<NewtonCollisionShape_Cone>(DEF_PHYSICS_CATEGORY.c_str());
        URHO3D_COPY_BASE_ATTRIBUTES(NewtonCollisionShape);
        URHO3D_ACCESSOR_ATTRIBUTE("Radius", GetRadius, SetRadius, float, 0.5f, AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("Length", GetLength, SetLength, float, 0.5f, AM_DEFAULT);

    }

    bool NewtonCollisionShape_Cone::buildNewtonCollision()
{
        //newtonShape_ = NewtonCreateCone(physicsWorld_->GetNewtonWorld(), radius_, length_, 0, nullptr);
		newtonShape_ = new ndShapeCone(radius_, length_);
        return true;
    }













	
//
//    NewtonCollisionShape_HeightmapTerrain::NewtonCollisionShape_HeightmapTerrain(Context* context) : NewtonCollisionShape_Geometry(context)
//    {
//        drawPhysicsDebugCollisionGeometry_ = false;//default newton debug lines for geometry is too many.
//    }
//
//    NewtonCollisionShape_HeightmapTerrain::~NewtonCollisionShape_HeightmapTerrain()
//    {
//
//    }
//
//    void NewtonCollisionShape_HeightmapTerrain::RegisterObject(Context* context)
//    {
//        context->RegisterFactory<NewtonCollisionShape_HeightmapTerrain>(DEF_PHYSICS_CATEGORY.c_str());
//        URHO3D_COPY_BASE_ATTRIBUTES(NewtonCollisionShape_Geometry);
//
//    }
//
//    bool NewtonCollisionShape_HeightmapTerrain::buildNewtonCollision()
//    {
//
//
//
//        //find a terrain component
//        Terrain* terrainComponent = node_->GetComponent<Terrain>();
//        if (terrainComponent)
//        {
//
//            int size = terrainComponent->GetHeightMap()->GetHeight();
//			float* heightDataRaw = terrainComponent->GetHeightData().get();
//			dFloat* heightData = (dFloat*)heightDataRaw;
//
//#ifdef _NEWTON_USE_DOUBLE
//            heightData = new dFloat[size*size];
//            for (int i = 0; i < size*size; i++) {
//                heightData[i] = heightDataRaw[i];
//            }
//#endif
//
//
//
//            char* const attibutes = new char[size * size];
//            memset(attibutes, 0, size * size * sizeof(char));
//
//            Vector3 spacing = terrainComponent->GetSpacing();
//            newtonShape_ = NewtonCreateHeightFieldCollision(physicsWorld_->GetNewtonWorld(), size, size, 0, 0, heightData, attibutes, 1.0f, spacing.x_, spacing.z_, 0);
//
//            delete[] attibutes;
//
//
//            ////set the internal offset correction to match where HeightmapTerrain renders
//            position_ = -Vector3(float(size*spacing.x_)*0.5f - spacing.x_*0.5f, 0, float(size*spacing.z_)*0.5f - spacing.z_*0.5f);
//            return true;
//        }
//        else
//            return false;
//    }
//
    NewtonCollisionShape_TreeCollision::NewtonCollisionShape_TreeCollision(Context* context) : NewtonCollisionShape_Geometry(context)
    {

    }

    NewtonCollisionShape_TreeCollision::~NewtonCollisionShape_TreeCollision()
    {

    }

    void NewtonCollisionShape_TreeCollision::RegisterObject(Context* context)
    {
        context->RegisterFactory<NewtonCollisionShape_TreeCollision>(DEF_PHYSICS_CATEGORY.c_str());
        URHO3D_COPY_BASE_ATTRIBUTES(NewtonCollisionShape_Geometry);
    }

    bool NewtonCollisionShape_TreeCollision::buildNewtonCollision()
    {
        if (!NewtonCollisionShape_Geometry::buildNewtonCollision())
            return false;

        ndWorld* world = physicsWorld_->GetNewtonWorld();
        

        newtonShape_.SetShape(new ndShapeStatic_bvh(newtonMesh_->newtonSoup));

        
        return true;
    }
//


}

