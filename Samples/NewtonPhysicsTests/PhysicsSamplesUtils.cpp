#include "PhysicsSamplesUtils.h"
#include "Urho3D/Graphics/Material.h"
#include "Urho3D/Graphics/Model.h"
#include "Urho3D/Graphics/StaticModel.h"
#include "Urho3D/Physics/RigidBody.h"
#include "Urho3D/Resource/ResourceCache.h"
#include "NewtonRigidBody.h"
#include "NewtonCollisionShapesDerived.h"

#define SAMPLE_MATERIAL "Materials/Metal.xml"
#define SAMPLE_COLOR (Vector4(0.5, 0.5f, 1.0f, 1.0f))

Node* SpawnSamplePhysicsSphere(Node* parentNode, const Vector3& worldPosition, float radius)
{
        Node* sphere1 = parentNode->CreateChild("SamplePhysicsSphere");
        Node* sphereVis = sphere1->CreateChild();
        sphereVis->SetScale(Vector3(radius, radius, radius)*2.0f);

        Model* sphereMdl = parentNode->GetSubsystem<ResourceCache>()->GetResource<Model>("Models/Sphere.mdl");
		SharedPtr<Material> sphereMat = parentNode->GetSubsystem<ResourceCache>()->GetResource<Material>(SAMPLE_MATERIAL)->Clone();
		sphereMat->SetShaderParameter("MatDiffColor", SAMPLE_COLOR);
		sphereMat->SetUVTransform(Vector2(), 0.0f, 1.0f/radius);

        StaticModel* sphere1StMdl = sphereVis->CreateComponent<StaticModel>();
        sphere1StMdl->SetCastShadows(true);
        sphere1StMdl->SetModel(sphereMdl);
        sphere1StMdl->SetMaterial(sphereMat);
        NewtonRigidBody* s1RigBody = sphere1->CreateComponent<NewtonRigidBody>();

        NewtonCollisionShape_Sphere* s1ColShape = sphere1->CreateComponent<NewtonCollisionShape_Sphere>();
        s1ColShape->SetScaleFactor(Vector3(radius*2.0f, radius*2.0f, radius*2.0f));

        sphere1->SetWorldPosition(worldPosition);

        s1RigBody->SetMassScale(1.0f);

        

        return sphere1;
}

Node* SpawnSamplePhysicsCylinder(Node* parentNode, const Vector3& worldPosition, float radius, float height)
{
    Node* sphere1 = parentNode->CreateChild("SamplePhysicsCylinder");
    Node* sphereVis = sphere1->CreateChild();
    sphereVis->SetScale(Vector3(radius*2.0f, height, radius*2.0f));

    Model* sphereMdl = parentNode->GetSubsystem<ResourceCache>()->GetResource<Model>("Models/Cylinder.mdl");
	SharedPtr<Material> sphereMat = parentNode->GetSubsystem<ResourceCache>()->GetResource<Material>(SAMPLE_MATERIAL)->Clone();
	sphereMat->SetShaderParameter("MatDiffColor", SAMPLE_COLOR);
	sphereMat->SetUVTransform(Vector2(), 0.0f, 1.0f / height);

    StaticModel* sphere1StMdl = sphereVis->CreateComponent<StaticModel>();
    sphere1StMdl->SetCastShadows(true);
    sphere1StMdl->SetModel(sphereMdl);
    sphere1StMdl->SetMaterial(sphereMat);

    NewtonRigidBody* s1RigBody = sphere1->CreateComponent<NewtonRigidBody>();

    NewtonCollisionShape_Cylinder* s1ColShape = sphere1->CreateComponent<NewtonCollisionShape_Cylinder>();
    s1ColShape->SetRadius1(radius);
    s1ColShape->SetRadius2(radius);
    s1ColShape->SetLength(height);

    //s1ColShape->SetScaleFactor(Vector3(height, radius*2, radius*2)*0.25f);

    s1ColShape->SetRotationOffset(Quaternion(0, 0, 90));
    sphere1->SetWorldPosition(worldPosition);

    s1RigBody->SetMassScale(1.0f);


    return sphere1;
}

Node* SpawnSamplePhysicsChamferCylinder(Node* parentNode, const Vector3& worldPosition, float radius, float height)
{
    Node* sphere1 = parentNode->CreateChild("SamplePhysicsCylinder");
    Node* sphereVis = sphere1->CreateChild();
    sphereVis->SetScale(Vector3(radius*2.0f, height, radius*2.0f));

    Model* sphereMdl = parentNode->GetSubsystem<ResourceCache>()->GetResource<Model>("Models/Cylinder.mdl");
	SharedPtr<Material> sphereMat = parentNode->GetSubsystem<ResourceCache>()->GetResource<Material>(SAMPLE_MATERIAL)->Clone();
	sphereMat->SetShaderParameter("MatDiffColor", SAMPLE_COLOR);
	sphereMat->SetUVTransform(Vector2(), 0.0f, 1.0f / height);


    StaticModel* sphere1StMdl = sphereVis->CreateComponent<StaticModel>();
    sphere1StMdl->SetCastShadows(true);
    sphere1StMdl->SetModel(sphereMdl);
    sphere1StMdl->SetMaterial(sphereMat);

    NewtonRigidBody* s1RigBody = sphere1->CreateComponent<NewtonRigidBody>();

    NewtonCollisionShape_ChamferCylinder* s1ColShape = sphere1->CreateComponent<NewtonCollisionShape_ChamferCylinder>();
    s1ColShape->SetRadius(radius);
    s1ColShape->SetLength(height);
    s1ColShape->SetRotationOffset(Quaternion(0, 0, 90));
    sphere1->SetWorldPosition(worldPosition);

    s1RigBody->SetMassScale(1.0f);


    return sphere1;
}

Node* SpawnSamplePhysicsCapsule(Node* parentNode, const Vector3& worldPosition, float radius, float height)
{
    Node* sphere1 = parentNode->CreateChild("SamplePhysicsCylinder");
    Node* sphereVis = sphere1->CreateChild();
    sphereVis->SetScale(Vector3(radius*2.0f, height, radius*2.0f));

    Model* sphereMdl = parentNode->GetSubsystem<ResourceCache>()->GetResource<Model>("Models/Cylinder.mdl");
	SharedPtr<Material> sphereMat = parentNode->GetSubsystem<ResourceCache>()->GetResource<Material>(SAMPLE_MATERIAL)->Clone();
	sphereMat->SetShaderParameter("MatDiffColor", SAMPLE_COLOR);
	sphereMat->SetUVTransform(Vector2(), 0.0f, 1.0f / height);

    StaticModel* sphere1StMdl = sphereVis->CreateComponent<StaticModel>();
    sphere1StMdl->SetCastShadows(true);
    sphere1StMdl->SetModel(sphereMdl);
    sphere1StMdl->SetMaterial(sphereMat);

    NewtonRigidBody* s1RigBody = sphere1->CreateComponent<NewtonRigidBody>();

    NewtonCollisionShape_Capsule* s1ColShape = sphere1->CreateComponent<NewtonCollisionShape_Capsule>();
    s1ColShape->SetRadius1(radius);
    s1ColShape->SetRadius2(radius);
    s1ColShape->SetLength(height);
    s1ColShape->SetRotationOffset(Quaternion(0, 0, 90));
    sphere1->SetWorldPosition(worldPosition);

    s1RigBody->SetMassScale(1.0f);


    return sphere1;
}


Node* SpawnSamplePhysicsCone(Node* parentNode, const Vector3& worldPosition, float radius, float height)
{
    Node* sphere1 = parentNode->CreateChild("SamplePhysicsCone");
    Node* sphereVis = sphere1->CreateChild();
    sphereVis->SetScale(Vector3(radius*2.0f, height, radius*2.0f));

    Model* sphereMdl = parentNode->GetSubsystem<ResourceCache>()->GetResource<Model>("Models/Cone.mdl");
	SharedPtr<Material> sphereMat = parentNode->GetSubsystem<ResourceCache>()->GetResource<Material>(SAMPLE_MATERIAL)->Clone();
	sphereMat->SetShaderParameter("MatDiffColor", SAMPLE_COLOR);
	sphereMat->SetUVTransform(Vector2(), 0.0f, 1.0f / height);

    StaticModel* sphere1StMdl = sphereVis->CreateComponent<StaticModel>();
    sphere1StMdl->SetCastShadows(true);
    sphere1StMdl->SetModel(sphereMdl);
    sphere1StMdl->SetMaterial(sphereMat);

    NewtonRigidBody* s1RigBody = sphere1->CreateComponent<NewtonRigidBody>();

    NewtonCollisionShape_Cone* s1ColShape = sphere1->CreateComponent<NewtonCollisionShape_Cone>();
    s1ColShape->SetRadius(radius);
    s1ColShape->SetLength(height);
    s1ColShape->SetRotationOffset(Quaternion(0, 0, 90));
    sphere1->SetWorldPosition(worldPosition);

    s1RigBody->SetMassScale(1.0f);


    return sphere1;
}



Node* SpawnSamplePhysicsBox(Node* parentNode, const Vector3& worldPosition, const Vector3& size)
{
    Node* box = parentNode->CreateChild();
    Node* boxVis = box->CreateChild();
    boxVis->SetScale(size);

    Model* sphereMdl = parentNode->GetSubsystem<ResourceCache>()->GetResource<Model>("Models/Box.mdl");
	SharedPtr<Material> sphereMat = parentNode->GetSubsystem<ResourceCache>()->GetResource<Material>(SAMPLE_MATERIAL)->Clone();
	sphereMat->SetShaderParameter("MatDiffColor", SAMPLE_COLOR);
	sphereMat->SetUVTransform(Vector2(), 0.0f, 1.0f / size.z_);

    StaticModel* sphere1StMdl = boxVis->CreateComponent<StaticModel>();
    sphere1StMdl->SetCastShadows(true);
    sphere1StMdl->SetModel(sphereMdl);
    sphere1StMdl->SetMaterial(sphereMat);


    NewtonRigidBody* s1RigBody = box->CreateComponent<NewtonRigidBody>();

    NewtonCollisionShape_Box* s1ColShape = box->CreateComponent<NewtonCollisionShape_Box>();
    s1ColShape->SetScaleFactor(size);

    box->SetWorldPosition(worldPosition);

    s1RigBody->SetMassScale(1.0f);


    return box;
}

