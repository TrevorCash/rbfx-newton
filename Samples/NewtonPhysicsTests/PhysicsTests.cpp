//
// Copyright (c) 2008-2019 the Urho3D project.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//

#include <Urho3D/Core/CoreEvents.h>
#include <Urho3D/Engine/Engine.h>
#include <Urho3D/Graphics/Camera.h>
#include <Urho3D/Graphics/DebugRenderer.h>
#include <Urho3D/Graphics/Graphics.h>
#include <Urho3D/Graphics/Light.h>
#include <Urho3D/Graphics/Material.h>
#include <Urho3D/Graphics/Model.h>
#include <Urho3D/Graphics/Octree.h>
#include <Urho3D/Graphics/Renderer.h>
#include <Urho3D/Graphics/Skybox.h>
#include <Urho3D/Graphics/Zone.h>
#include <Urho3D/Input/Input.h>
#include <Urho3D/IO/File.h>
#include <Urho3D/IO/FileSystem.h>
#include <NewtonCollisionShape.h>
#include <NewtonCollisionShapesDerived.h>
#include "Urho3D/Physics/PhysicsWorld.h"
#include <Urho3D/Physics/RigidBody.h>
#include <Urho3D/Resource/ResourceCache.h>
#include <Urho3D/Scene/Scene.h>
#include <Urho3D/UI/Font.h>
#include <Urho3D/UI/Text.h>
#include <Urho3D/UI/UI.h>
#include "PhysicsTests.h"
#include "PhysicsSamplesUtils.h"

#include <Urho3D/DebugNew.h>
#include "NewtonFixedDistanceConstraint.h"
#include "NewtonBallAndSocketConstraint.h"
#include "NewtonKinematicsJoint.h"
#include "NewtonFullyFixedConstraint.h"
#include "NewtonHingeConstraint.h"
#include "NewtonSliderConstraint.h"
#include "NewtonPhysicsEvents.h"
#include "Urho3D/UI/Text3D.h"

#include "Urho3D/Graphics/Terrain.h"
#include "NewtonPhysicsWorld.h"
#include "Urho3D/Scene/Node.h"
#include "NewtonRigidBody.h"


URHO3D_DEFINE_APPLICATION_MAIN(PhysicsTests)

PhysicsTests::PhysicsTests(Context* context) :
    Sample(context)
{
}

void PhysicsTests::Start()
{
    // Execute base class startup
    Sample::Start();

	//Register Newton Physics Lib
	RegisterNewtonPhysicsLibrary(context_);


    // Create the scene content
    CreateScene();

    // Create the UI content
    CreateInstructions();

    // Setup the viewport for displaying the scene
    SetupViewport();

    // Hook up to the frame update and render post-update events
    SubscribeToEvents();

    // Set the mouse mode to use in the sample
    Sample::InitMouseMode(MM_RELATIVE);
}

void PhysicsTests::CreateScene()
{
    auto* cache = GetSubsystem<ResourceCache>();

    scene_ = new Scene(context_);

    scene_->SetTimeScale(1.0f);

    // Create octree, use default volume (-1000, -1000, -1000) to (1000, 1000, 1000)
    // Create a physics simulation world with default parameters, which will update at 60fps. the Octree must
    // exist before creating drawable components, the PhysicsWorld must exist before creating physics components.
    // Finally, create a DebugRenderer component so that we can draw physics debug geometry
    scene_->CreateComponent<Octree>();
    NewtonPhysicsWorld* newtonWorld = scene_->CreateComponent<NewtonPhysicsWorld>();
    newtonWorld->SetGravity(Vector3(0, -9.81f, 0));
    //scene_->CreateComponent<NewtonCollisionShape_SceneCollision>();
    scene_->CreateComponent<DebugRenderer>();

    // Create a Zone component for ambient lighting & fog control
    Node* zoneNode = scene_->CreateChild("Zone");
    auto* zone = zoneNode->CreateComponent<Zone>();
    zone->SetBoundingBox(BoundingBox(-1000.0f, 1000.0f));
    zone->SetAmbientColor(Color(0.15f, 0.15f, 0.15f));
    zone->SetFogColor(Color(1.0f, 1.0f, 1.0f));
    zone->SetFogStart(300.0f);
    zone->SetFogEnd(500.0f);

    // Create a directional light to the world. Enable cascaded shadows on it
    Node* lightNode = scene_->CreateChild("DirectionalLight");
    lightNode->SetDirection(Vector3(0.6f, -1.0f, 0.8f));
    auto* light = lightNode->CreateComponent<Light>();
    light->SetLightType(LIGHT_DIRECTIONAL);
    light->SetCastShadows(true);
    light->SetShadowBias(BiasParameters(0.00025f, 0.5f));
    // Set cascade splits at 10, 50 and 200 world units, fade shadows out at 80% of maximum shadow distance
    light->SetShadowCascade(CascadeParameters(10.0f, 50.0f, 200.0f, 0.0f, 0.8f));

    // Create skybox. The Skybox component is used like StaticModel, but it will be always located at the camera, giving the
    // illusion of the box planes being far away. Use just the ordinary Box model and a suitable material, whose shader will
    // generate the necessary 3D texture coordinates for cube mapping
    Node* skyNode = scene_->CreateChild("Sky");
    skyNode->SetScale(500.0f); // The scale actually does not matter
    auto* skybox = skyNode->CreateComponent<Skybox>();
    skybox->SetModel(cache->GetResource<Model>("Models/Box.mdl"));
    skybox->SetMaterial(cache->GetResource<Material>("Materials/Skybox.xml"));



    CreateScenery(Vector3(0,0,0));

    //SpawnSamplePhysicsCylinder(scene_, Vector3(-5, 2, 0));
    //SpawnSamplePhysicsCylinder(scene_, Vector3(5, 2, 0), 0.25f,4);


    //SpawnMaterialsTest(Vector3(0,-25,100));


    //SpawnCompoundedRectTest2(Vector3(0, 2, 0));

    //SpawnBallSocketTest(Vector3(50, 10, 0));
    //SpawnHingeActuatorTest(Vector3(52, 10, 0));

    //CreatePyramids(Vector3(0,0,0));


    //SpawnCompound(Vector3(-2, 10 , 10));
    //SpawnConvexHull(Vector3(-2, 3, 10));

    Quaternion tilt = Quaternion(Random(-1.0f, 1.0f), Vector3(1, 0, 0));

    //SpawnTrialBike(Vector3(5, 5, 0),  Quaternion(0, Vector3(0, 1, 0)) * tilt, true);
    //SpawnTrialBike(Vector3(-5, 5, 0), Quaternion(90, Vector3(0, 1, 0)) * tilt, false);

    //SpawnKinematicBodyTest(Vector3(0, 0, 0), Quaternion::IDENTITY);


    //SpawnHingeSpringTest(Vector3(0,10,0), Quaternion::IDENTITY);
    //SpawnHingeSpringTest(Vector3(-2, 10, 0), Quaternion(-90, Vector3(0,1,0)));

    //SpawnCollisionExceptionsTest(Vector3(0, 1, 15));

    //SpawnSliderTest(Vector3(0, 10, 10));
    //SpawnLinearJointedObject(1.0f, Vector3(10 , 2, 10));

    SpawnNSquaredJointedObject(Vector3(-20, 20, 10));

    //SpawnCompoundedRectTest(Vector3(20, 10, 10));

	//SpawnRejointingTest(Vector3(0, 10, 0));

    ////////create scale test
    //SpawnSceneCompoundTest(Vector3(-20, 10, 20), true);
    //SpawnSceneCompoundTest(Vector3(-20, 10, 30), false); //this was gives newton a non-orthogonal matrix.

    //CreateTowerOfLiar(Vector3(40, 0, 20));

    //

    // Create the camera. Set far clip to match the fog. Note: now we actually create the camera node outside the scene, because
    // we want it to be unaffected by scene load / save
    cameraNode_ = new Node(context_);
    auto* camera = cameraNode_->CreateComponent<Camera>();
    camera->SetFarClip(500.0f);

    // Set an initial position for the camera scene node above the floor
    cameraNode_->SetPosition(Vector3(0.0f, 5.0f, -15.0));
}
void PhysicsTests::CreateInstructions()
{
    auto* cache = GetSubsystem<ResourceCache>();
    auto* ui = GetSubsystem<UI>();

    // Construct new Text object, set string to display and font to use
    auto* instructionText = ui->GetRoot()->CreateChild<Text>();
    instructionText->SetText(
        "Use WASD keys and mouse/touch to move\n"
        "LMB to spawn physics objects\n"
        "F5 to save scene, F7 to load\n"
        "Space to toggle physics debug geometry"
    );
    instructionText->SetFont(cache->GetResource<Font>("Fonts/Anonymous Pro.ttf"), 15);
    // The text has multiple rows. Center them in relation to each other
    instructionText->SetTextAlignment(HA_CENTER);

    // Position the text relative to the screen center
    instructionText->SetHorizontalAlignment(HA_CENTER);
    instructionText->SetVerticalAlignment(VA_CENTER);
    instructionText->SetPosition(0, ui->GetRoot()->GetHeight() / 4);
}

void PhysicsTests::SetupViewport()
{
    auto* renderer = GetSubsystem<Renderer>();

    // Set up a viewport to the Renderer subsystem so that the 3D scene can be seen
    SharedPtr<Viewport> viewport(new Viewport(context_, scene_, cameraNode_->GetComponent<Camera>()));
    renderer->SetViewport(0, viewport);
}

void PhysicsTests::SubscribeToEvents()
{
    // Subscribe HandleUpdate() function for processing update events
    SubscribeToEvent(E_UPDATE, URHO3D_HANDLER(PhysicsTests, HandleUpdate));

    // Subscribe HandlePostRenderUpdate() function for processing the post-render update event, during which we request
    // debug geometry
    SubscribeToEvent(E_POSTRENDERUPDATE, URHO3D_HANDLER(PhysicsTests, HandlePostRenderUpdate));

    SubscribeToEvent(E_MOUSEBUTTONUP, URHO3D_HANDLER(PhysicsTests, HandleMouseButtonUp));

    SubscribeToEvent(E_MOUSEBUTTONDOWN, URHO3D_HANDLER(PhysicsTests, HandleMouseButtonDown));


    SubscribeToEvent(E_NEWTON_PHYSICSCOLLISIONSTART, URHO3D_HANDLER(PhysicsTests, HandleCollisionStart));
    SubscribeToEvent(E_NEWTON_PHYSICSCOLLISION, URHO3D_HANDLER(PhysicsTests, HandleCollision));
    SubscribeToEvent(E_NEWTON_PHYSICSCOLLISIONEND, URHO3D_HANDLER(PhysicsTests, HandleCollisionEnd));


    SubscribeToEvent(E_NEWTON_PHYSICSPRESTEP, URHO3D_HANDLER(PhysicsTests, HandlePhysicsPreStep));
    SubscribeToEvent(E_NEWTON_PHYSICSPOSTSTEP, URHO3D_HANDLER(PhysicsTests, HandlePhysicsPostStep));
}

void PhysicsTests::MoveCamera(float timeStep)
{
    // Do not move if the UI has a focused element (the console)
    if (GetSubsystem<UI>()->GetFocusElement())
        return;

    auto* input = GetSubsystem<Input>();

    // Movement speed as world units per second
    const float MOVE_SPEED = 20.0f;
    // Mouse sensitivity as degrees per pixel
    const float MOUSE_SENSITIVITY = 0.1f;

    // Use this frame's mouse motion to adjust camera node yaw and pitch. Clamp the pitch between -90 and 90 degrees
    IntVector2 mouseMove = input->GetMouseMove();
    yaw_ += MOUSE_SENSITIVITY * mouseMove.x_;
    pitch_ += MOUSE_SENSITIVITY * mouseMove.y_;
    pitch_ = Clamp(pitch_, -90.0f, 90.0f);

    // Construct new orientation for the camera scene node from yaw and pitch. Roll is fixed to zero
    cameraNode_->SetRotation(Quaternion(pitch_, yaw_, 0.0f));

    float speedFactor = 1.0f;
    if (input->GetKeyDown(KEY_SHIFT) && !input->GetKeyDown(KEY_CTRL))
        speedFactor *= 0.25f;

    if (!input->GetKeyDown(KEY_SHIFT) && input->GetKeyDown(KEY_CTRL))
        speedFactor *= 10.0f;

    // Read WASD keys and move the camera scene node to the corresponding direction if they are pressed
    if (input->GetKeyDown(KEY_W))
        cameraNode_->Translate(Vector3::FORWARD * MOVE_SPEED * speedFactor * timeStep);
    if (input->GetKeyDown(KEY_S))
        cameraNode_->Translate(Vector3::BACK * MOVE_SPEED * speedFactor *timeStep);
    if (input->GetKeyDown(KEY_A))
        cameraNode_->Translate(Vector3::LEFT * MOVE_SPEED * speedFactor * timeStep);
    if (input->GetKeyDown(KEY_D))
        cameraNode_->Translate(Vector3::RIGHT * MOVE_SPEED * speedFactor *timeStep);



    if (input->GetMouseButtonPress(MOUSEB_LEFT))
        CreatePickTargetNodeOnPhysics();



    if (input->GetKeyPress(KEY_R)) {

        //Perform a physics RayCast in camera direction
    }

    if (input->GetKeyPress(KEY_TAB))
    {
        input->SetMouseMode(MM_ABSOLUTE);
        GetSubsystem<Input>()->SetMouseVisible(!GetSubsystem<Input>()->IsMouseVisible());
        GetSubsystem<Input>()->SetMouseGrabbed(!GetSubsystem<Input>()->IsMouseGrabbed());
       
    }


   // if (input->GetMouseButtonPress(MOUSEB_RIGHT))
    //    DecomposePhysicsTree();

    if (input->GetMouseButtonPress(MOUSEB_RIGHT))
    {
        SpawnRandomObjects();
    }

    if (input->GetKeyPress(KEY_T))
        TransportNode();

    if (input->GetKeyPress(KEY_Y))
        RecomposePhysicsTree();

    if (input->GetKeyPress(KEY_DELETE))
    {
        //if()
        RemovePickNode(input->GetKeyDown(KEY_SHIFT));
    }

    if (input->GetKeyPress(KEY_PERIOD))
    {
        //do a raycast test
        ea::vector<NewtonRigidBody*> bodies;

        float length = M_LARGE_VALUE;

        Ray ray = Ray(cameraNode_->GetWorldPosition(), cameraNode_->GetWorldDirection());
        //GSS<VisualDebugger>()->AddLine(ray.origin_, ray.origin_ + ray.direction_ * length, Color::RED)->SetLifeTimeMs(100000);
		ea::vector<PhysicsRayCastIntersection> intersections;
        scene_->GetComponent<NewtonPhysicsWorld>()->RayCast(intersections, ray);

        for (PhysicsRayCastIntersection& intersection : intersections) {
           // PrintPhysicsRayCastIntersection(intersection);

            //GetSubsystem<VisualDebugger>()->AddCross(intersection.rayIntersectWorldPosition_, 0.125f, Color::RED, false)->SetLifeTimeMs(100000);
            //GetSubsystem<VisualDebugger>()->AddLine(intersection.rayIntersectWorldPosition_, intersection.rayIntersectWorldPosition_ + intersection.rayIntersectWorldNormal_, Color::RED, false)->SetLifeTimeMs(100000);



        }
    }


    if (input->GetKeyPress(KEY_L))
    {
        //mark all physics things dirty
        ea::vector<Node*> nodes;
        scene_->GetChildrenWithComponent<NewtonRigidBody>(nodes, true);

        for (Node* node : nodes)
        {
            node->GetComponent<NewtonRigidBody>()->MarkDirty();
        }
        nodes.clear();

        scene_->GetChildren(nodes, true);

        for (Node* node : nodes)
        {
            if(node->GetDerivedComponent<NewtonCollisionShape>())
                node->GetDerivedComponent<NewtonCollisionShape>()->MarkDirty();
        }
    }

    // Check for loading/saving the scene. Save the scene to the file Data/Scenes/Physics.xml relative to the executable
        // directory
    if (input->GetKeyPress(KEY_F5))
    {
        ea::string filePath = GetSubsystem<FileSystem>()->GetProgramDir();
#if _MSC_VER
        filePath += "../";
#endif
        filePath += "Data/Scenes/PhysicsTests.xml";
        File saveFile(context_, filePath, FILE_WRITE);
        scene_->SaveXML(saveFile);
    }
    if (input->GetKeyPress(KEY_F7))
    {
		ea::string filePath = GetSubsystem<FileSystem>()->GetProgramDir();
#if _MSC_VER
        filePath += "../";
#endif
        filePath += "Data/Scenes/PhysicsTests.xml";
        File loadFile(context_, filePath, FILE_READ);
        scene_->LoadXML(loadFile);
    }

    if (input->GetKeyPress(KEY_R) && input->GetKeyDown(KEY_SHIFT))
    {
        scene_->Clear();
        Stop();
        Start();
    }



    // Toggle physics debug geometry with space
    if (input->GetKeyPress(KEY_SPACE))
        drawDebug_ = !drawDebug_;


    if (0) {
        //do a collision cast around camera
        ea::vector<NewtonRigidBody*> bodies;
        scene_->GetComponent<NewtonPhysicsWorld>()->GetRigidBodies(bodies, Sphere(cameraNode_->GetWorldPosition(), 2.0f));

        if (bodies.size())
            URHO3D_LOGINFO(ea::to_string(bodies.size()));
    }

    if (0)
    {
        //test collision point on convex hull
        Node* convexHull = scene_->GetChild("convexhull", true);
        
        if (convexHull)
        {

            bool s = scene_->GetComponent<NewtonPhysicsWorld>()->RigidBodyContainsPoint(convexHull->GetComponent<NewtonRigidBody>(), cameraNode_->GetWorldPosition());


            if (s)
                URHO3D_LOGINFO("collision!");
        }

    }



	if (input->GetKeyPress(KEY_4))
	{
		ToggleRejointTest();

	}



}



void PhysicsTests::SpawnSceneCompoundTest(const Vector3& worldPos, bool oneBody)
{
    Node* root = scene_->CreateChild();
    root->SetPosition(worldPos);
    const int levelCount = 10;
    const int breadth = 2;
    Node* curNode = root;


    for (int i = 0; i < levelCount; i++)
    {

        curNode = curNode->CreateChild();
        curNode->SetName("SpawnSceneCompoundTest:" + ea::to_string(i));
        curNode->AddTag("scaleTestCube");
        float rotDelta = 10.0f;

        curNode->SetScale(Vector3(Random(0.8f, 1.2f), Random(0.8f, 1.2f), Random(0.8f, 1.2f)));
        curNode->Rotate(Quaternion(Random(-rotDelta, rotDelta), Random(-rotDelta, rotDelta), Random(-rotDelta, rotDelta)));
        curNode->Translate(Vector3(Random(0.5f, 2.0f), Random(0.5f,2.0f), Random(0.5f, 2.0f)));

        StaticModel* stMdl = curNode->CreateComponent<StaticModel>();
        stMdl->SetModel(GetSubsystem<ResourceCache>()->GetResource<Model>("Models/Cone.mdl"));
        stMdl->SetMaterial(GetSubsystem<ResourceCache>()->GetResource<Material>("Materials/Stone.xml"));
        stMdl->SetCastShadows(true);
        if (i == 0 || !oneBody) {
            NewtonRigidBody* rigBody = curNode->CreateComponent<NewtonRigidBody>();
            rigBody->SetMassScale(1.0f);
            //rigBody->SetAngularDamping(1.0f);
        }
        NewtonCollisionShape* colShape = curNode->CreateComponent<NewtonCollisionShape_Cone>();
        colShape->SetRotationOffset(Quaternion(0, 0, 90));

        //URHO3D_LOGINFO(String(curNode->GetWorldTransform().HasSkew()));

    }
}



void PhysicsTests::SpawnObject()
{
    auto* cache = GetSubsystem<ResourceCache>();
    Node* firstNode = nullptr;
    Node* prevNode = nullptr;
    bool isFirstNode = true;
    for (int i = 0; i < 2; i++) {


        // Create a smaller box at camera position
        Node* boxNode;
        if (prevNode)
            boxNode = prevNode->CreateChild();
        else {
            boxNode = scene_->CreateChild();
            firstNode = boxNode;
        }
        prevNode = boxNode;
        const float range = 3.0f;


        boxNode->SetWorldPosition(cameraNode_->GetWorldPosition() + Vector3(Random(-1.0f,1.0f) * range,Random(-1.0f, 1.0f)* range, Random(-1.0f, 1.0f)* range));
        boxNode->SetRotation(cameraNode_->GetRotation());
        boxNode->SetScale(1.0f);

        auto* boxObject = boxNode->CreateComponent<StaticModel>();
        boxObject->SetModel(cache->GetResource<Model>("Models/Box.mdl"));
        boxObject->SetMaterial(cache->GetResource<Material>("Materials/StoneEnvMapSmall.xml"));
        boxObject->SetCastShadows(true);


         // Create physics components, use a smaller mass also
         auto* body = boxNode->CreateComponent<NewtonRigidBody>();
         body->SetMassScale(0.1f);
        
        

        isFirstNode = false;
        //body->SetContinuousCollision(true);
        auto* shape = boxNode->CreateComponent<NewtonCollisionShape_Box>();

        const float OBJECT_VELOCITY = 20.0f;

        // Set initial velocity for the RigidBody based on camera forward vector. Add also a slight up component
        // to overcome gravity better
       // body->SetLinearVelocity(cameraNode_->GetRotation() * Vector3(0.0f, 0.25f, 1.0f) * OBJECT_VELOCITY);

    }

}


void PhysicsTests::CreatePyramids(Vector3 position)
{
    int size = 8;
    float horizontalSeperation = 2.0f;
    //create pyramids
    const int numIslands = 0;
    for (int x2 = -numIslands; x2 <= numIslands; x2++)
        for (int y2 = -numIslands; y2 <= numIslands; y2++)
        {
            for (int y = 0; y < size; ++y)
            {
                for (int x = -y; x <= y; ++x)
                {
                    Node* node = SpawnSamplePhysicsBox(scene_, Vector3((float)x*horizontalSeperation, -(float)y + float(size), 0.0f) + Vector3(x2, 0, y2)*50.0f + position, Vector3::ONE);
                }
            }
        }
}


void PhysicsTests::CreateTowerOfLiar(Vector3 position)
{
    float length = 10.0f;
    float width = 5.0f;
    int numBoxes = 16;

    float thickness = 10.0f / (float(numBoxes));
    float fudgeFactor = 0.04f;
    Vector3 curPosition = position - Vector3(0,thickness*0.5f,0);
    for (int i = 0; i < numBoxes; i++) {
        float delta = length / (2.0f*(numBoxes - i));


        curPosition = curPosition + Vector3(delta - delta*fudgeFactor, thickness, 0);

        Node* box = SpawnSamplePhysicsBox(scene_, curPosition, Vector3(length, thickness, width));
        //box->GetComponent<RigidBody>()->SetAutoSleep(false);

  
    }




}





void PhysicsTests::SpawnConvexHull(const Vector3& worldPos)
{
    auto* cache = GetSubsystem<ResourceCache>();

    // Create a smaller box at camera position
    Node* boxNode = scene_->CreateChild("convexhull");

    boxNode->SetWorldPosition(worldPos);
    boxNode->SetScale(1.0f);

    auto* boxObject = boxNode->CreateComponent<StaticModel>();
    boxObject->SetModel(cache->GetResource<Model>("Models/Mushroom.mdl"));
    boxObject->SetMaterial(cache->GetResource<Material>("Materials/Mushroom.xml"));
    boxObject->SetCastShadows(true);


    // Create physics components
    auto* body = boxNode->CreateComponent<NewtonRigidBody>();
    body->SetMassScale(1.0f);

    auto* shape = boxNode->CreateComponent<NewtonCollisionShape_ConvexHull>();


    boxNode->Rotate(Quaternion(90, Vector3(1, 1, 0)));
    URHO3D_LOGINFO("Initial Position: " + (boxNode->GetWorldPosition().ToString()));
}



void PhysicsTests::SpawnCompound(const Vector3& worldPos)
{
    auto* cache = GetSubsystem<ResourceCache>();

    // Create a smaller box at camera position
    Node* boxNode = scene_->CreateChild();

    boxNode->SetWorldPosition(worldPos);
    boxNode->SetScale(1.0f);

    auto* boxObject = boxNode->CreateComponent<StaticModel>();
    boxObject->SetModel(cache->GetResource<Model>("Models/Mushroom.mdl"));
    boxObject->SetMaterial(cache->GetResource<Material>("Materials/Mushroom.xml"));
    boxObject->SetCastShadows(true);


    // Create physics components, use a smaller mass also
    auto* body = boxNode->CreateComponent<NewtonRigidBody>();
    body->SetMassScale(1.0f);

    auto* shape = boxNode->CreateComponent<NewtonCollisionShape_ConvexHullCompound>();


    boxNode->Rotate(Quaternion(90, Vector3(1, 1, 0)));



}




void PhysicsTests::SpawnDecompCompound(const Vector3& worldPos)
{
    auto* cache = GetSubsystem<ResourceCache>();

    // Create a smaller box at camera position
    Node* boxNode = scene_->CreateChild();

    boxNode->SetWorldPosition(worldPos);
    boxNode->SetScale(1.0f);

    auto* boxObject = boxNode->CreateComponent<StaticModel>();
    boxObject->SetModel(cache->GetResource<Model>("Models/Mushroom.mdl"));
    boxObject->SetMaterial(cache->GetResource<Material>("Materials/Mushroom.xml"));
    boxObject->SetCastShadows(true);


    // Create physics components, use a smaller mass also
    auto* body = boxNode->CreateComponent<NewtonRigidBody>();
    body->SetMassScale(1.0f);

    auto* shape = boxNode->CreateComponent<NewtonCollisionShape_ConvexDecompositionCompound>();
}

void PhysicsTests::SpawnNSquaredJointedObject(Vector3 worldPosition)
{
    //lets joint spheres together with a distance limiting joint.
    const float dist = 5.0f;

    const int numSpheres = 20;

    ea::vector<Node*> nodes;
    //make lots of spheres
    for (int i = 0; i < numSpheres; i++)
    {
        Node* node = SpawnSamplePhysicsSphere(scene_, worldPosition  + Vector3(0,dist*0.5f,0) - Quaternion(Random()*360.0f, Random()*360.0f, Random()*360.0f) * (Vector3::FORWARD*dist));
      
        nodes.push_back(node);
        
    }


    //connect them all O(n*n) joints
    for (Node* node : nodes)
    {
        for (Node* node2 : nodes)
        {
            if (node == node2)
                continue;

            NewtonFixedDistanceConstraint* constraint = node->CreateComponent<NewtonFixedDistanceConstraint>();
            constraint->SetOwnRotation(Quaternion(45, 45, 45));
            //constraint->SetOtherRotation(Quaternion(45, 0, 0));
            constraint->SetOtherBody(node2->GetComponent<NewtonRigidBody>());
            constraint->SetOtherPosition(Vector3(0.0, 0, 0));
            constraint->SetEnableForceCalculation(true);

            forceCalculationConstraints_.push_back(constraint);
        }
    }
}

void PhysicsTests::SpawnGlueJointedObject(Vector3 worldPosition)
{
    //lets joint spheres together with a distance limiting joint.
    const float dist = 10.0f;

    const int numSpheres = 25;

    ea::vector<Node*> nodes;
    //make lots of spheres
    for (int i = 0; i < numSpheres; i++)
    {
        Node* node = SpawnSamplePhysicsSphere(scene_, worldPosition + Vector3(0, dist*0.5f, 0) - Quaternion(Random()*360.0f, Random()*360.0f, Random()*360.0f) * (Vector3::FORWARD*dist));

        nodes.push_back(node);

    }


    //connect them all O(n*n) joints
    for (Node* node : nodes)
    {
        if (node == nodes[0])
            continue;

        NewtonFullyFixedConstraint* constraint = node->CreateComponent<NewtonFullyFixedConstraint>();
        constraint->SetOtherBody(nodes[0]->GetComponent<NewtonRigidBody>());
        constraint->SetOtherPosition(Vector3(0.0, 0, 0));

    }
}



void PhysicsTests::SpawnLinearJointedObject(float size, Vector3 worldPosition)
{
    //lets joint spheres together with a distance limiting joint.
    const float dist = size;

    const int numSpheres = 2;

	ea::vector<Node*> nodes;
    //make lots of spheres
    for (int i = 0; i < numSpheres; i++)
    {
		nodes.push_back(SpawnSamplePhysicsSphere(scene_, worldPosition + Vector3(0, i*dist, 0), dist*0.5f));

        if (i > 0) {
            NewtonHingeConstraint* constraint = nodes[i - 1]->CreateComponent<NewtonHingeConstraint>();
            constraint->SetOtherBody(nodes[i]->GetComponent<NewtonRigidBody>());
            constraint->SetWorldPosition(worldPosition + Vector3(0, i*dist, 0) - Vector3(0, dist, 0)*0.5f);
            //constraint->SetOwnRotation(Quaternion(0, 0, -90));
           // constraint->SetOtherRotation(Quaternion(0,0,-90));
           // constraint->SetTwistLimitsEnabled(true);
            
        }
    }
}



void PhysicsTests::SpawnMaterialsTest(Vector3 worldPosition)
{

    Node* ramp = SpawnSamplePhysicsBox(scene_, worldPosition, Vector3(100, 1, 100));
    ramp->Rotate(Quaternion(-20.0f, 0, 0));
    ramp->Translate(Vector3(0, 50, 0), TS_WORLD);
    ramp->GetComponent<NewtonRigidBody>()->SetMassScale(0);


    for (int i = 0; i < 5; i++)
    {
        Node* box = SpawnSamplePhysicsBox(scene_, ramp->GetWorldPosition() + Vector3(-2.5 + float(i)*1.1f, 2, 0), Vector3::ONE);


        NewtonCollisionShape* collisionShape = box->GetDerivedComponent<NewtonCollisionShape>();

        collisionShape->SetStaticFriction(0.1f*i - 0.05f);
        collisionShape->SetKineticFriction(0.1f*i);
      
    }

    SpawnCompoundedRectTest(ramp->GetWorldPosition() + Vector3(-5, 8, 10));

}



void PhysicsTests::SpawnBallSocketTest(Vector3 worldPosition)
{
    //lets joint spheres together with a distance limiting joint.

    Node* sphere1 =  SpawnSamplePhysicsSphere(scene_, worldPosition);
    Node* sphere2 = SpawnSamplePhysicsSphere(scene_, worldPosition + Vector3(0,2.0, 0));
    //sphere1->GetComponent<RigidBody>()->SetMassScale(0);
    NewtonBallAndSocketConstraint* constraint = sphere1->CreateComponent<NewtonBallAndSocketConstraint>();


    constraint->SetOtherWorldPosition(sphere2->GetWorldPosition() - Vector3(0, 2.0, 0));
    constraint->SetOtherBody(sphere2->GetComponent<NewtonRigidBody>());

    //sphere2->GetComponent<RigidBody>()->SetMassScale(0.0f);
    //sphere1->GetComponent<RigidBody>()->SetMassScale(0.0f);


}
void PhysicsTests::SpawnHingeActuatorTest(Vector3 worldPosition)
{
    //lets joint spheres together with a distance limiting joint.

    Node* box1 = SpawnSamplePhysicsBox(scene_, worldPosition, Vector3(10,1,10));
    Node* box2 = SpawnSamplePhysicsBox(scene_, worldPosition + Vector3(10, 0, 0), Vector3(10, 1, 10));

    //box1->GetComponent<RigidBody>()->SetAutoSleep(false);
   // box2->GetComponent<RigidBody>()->SetAutoSleep(false);


    //sphere1->GetComponent<RigidBody>()->SetMassScale(0);
    NewtonHingeConstraint* constraint = box1->CreateComponent<NewtonHingeConstraint>();
    constraint->SetWorldPosition(worldPosition + Vector3(10, 1, 0)*0.5f);
    constraint->SetWorldRotation(Quaternion(0, 90, 0));
    //constraint->SetOtherWorldPosition(sphere2->GetWorldPosition() - Vector3(0, 2.0, 0));
    constraint->SetOtherBody(box2->GetComponent<NewtonRigidBody>());


    constraint->SetPowerMode(NewtonHingeConstraint::ACTUATOR);
    constraint->SetMaxTorque(10000.0f);
   // constraint->SetEnableLimits(false);
    constraint->SetActuatorMaxAngularRate(1000.0f);
    constraint->SetActuatorTargetAngle(0.0f);
    hingeActuatorTest = constraint;
    //sphere2->GetComponent<RigidBody>()->SetMassScale(0.0f);
    //sphere1->GetComponent<RigidBody>()->SetMassScale(0.0f);


}



void PhysicsTests::SpawnCollisionExceptionsTest(Vector3 worldPosition)
{

    Node* a = SpawnSamplePhysicsBox(scene_, worldPosition, Vector3(1, 1, 1));
    Node* b = SpawnSamplePhysicsBox(scene_, worldPosition + Vector3(0,1,0), Vector3(1, 1, 1)*0.9f);
    Node* c = SpawnSamplePhysicsBox(scene_, worldPosition + Vector3(0, 1, 0)*2, Vector3(1, 1, 1)*0.8f);
    Node* d = SpawnSamplePhysicsBox(scene_, worldPosition + Vector3(0, 1, 0)*3, Vector3(1, 1, 1)*0.7f);
    Node* e = SpawnSamplePhysicsBox(scene_, worldPosition + Vector3(0, 1, 0)*4, Vector3(1, 1, 1)*0.5f);

    NewtonRigidBody* a_b = a->GetComponent<NewtonRigidBody>();
    NewtonRigidBody* b_b = b->GetComponent<NewtonRigidBody>();
    NewtonRigidBody* c_b = c->GetComponent<NewtonRigidBody>();
    NewtonRigidBody* d_b = d->GetComponent<NewtonRigidBody>();
    NewtonRigidBody* e_b = e->GetComponent<NewtonRigidBody>();


    URHO3D_LOGINFO(ea::to_string(a_b->GetID()));
    URHO3D_LOGINFO(ea::to_string(b_b->GetID()));
    URHO3D_LOGINFO(ea::to_string(c_b->GetID()));

    a_b->SetCollisionOverride(e_b, false);

    a_b->SetCollisionOverride(d_b, false);

    a_b->SetCollisionOverride(c_b, false);

    c_b->SetNoCollideOverride(true);

}

void PhysicsTests::SpawnSliderTest(Vector3 worldPosition)
{
    Node* a = SpawnSamplePhysicsBox(scene_, worldPosition, Vector3::ONE);
    Node* b = SpawnSamplePhysicsBox(scene_, worldPosition+Vector3(1,0,0), Vector3::ONE);
    //a->GetComponent<RigidBody>()->SetMassScale(0.0f);
    

    NewtonSliderConstraint* constraint = a->CreateComponent<NewtonSliderConstraint>();
    constraint->SetOtherBody(b->GetComponent<NewtonRigidBody>());

    constraint->SetEnableSliderLimits(true, true);
    constraint->SetSliderLimits(-2, 2);

   // constraint->SetEnableTwistLimits(true, true);
    //constraint->SetTwistLimits(-180, 180);




    constraint->SetEnableSliderSpringDamper(true);
    //constraint->SetEnableTwistSpringDamper(true);

}

void PhysicsTests::SpawnRandomObjects()
{
    float range = 10.0f;

    for (int i = 0; i < 100; i++) {

        Vector3 posOffset = Vector3(Random(-range, range), Random(-range, range), Random(-range, range));
        int ran = Random(4);
        Node* node = nullptr;
        if (ran == 0)
            node = SpawnSamplePhysicsSphere(scene_, cameraNode_->GetWorldPosition() + posOffset);
        else if (ran == 1)
            node = SpawnSamplePhysicsBox(scene_, cameraNode_->GetWorldPosition() + posOffset, Vector3::ONE);
        else if (ran == 2)
            node = SpawnSamplePhysicsCone(scene_, cameraNode_->GetWorldPosition() + posOffset, 0.5f);
        else
            node = SpawnSamplePhysicsCylinder(scene_, cameraNode_->GetWorldPosition() + posOffset, 0.5f);


        node->GetComponent<NewtonRigidBody>()->SetLinearVelocity(cameraNode_->GetWorldDirection() * 10.0f);
        node->GetComponent<NewtonRigidBody>()->SetContinuousCollision(false);
        //node->GetComponent<RigidBody>()->SetLinearDamping(0.01f);
        //node->GetComponent<RigidBody>()->SetMassScale(Random(1.0f, 10.0f));
        node->GetComponent<NewtonRigidBody>()->SetGenerateContacts(false);

    }

}

void PhysicsTests::SpawnCompoundedRectTest(Vector3 worldPosition)
{
    //make 2 1x1x1 physics rectangles. 1 with just one shape and 1 with 2 smaller compounds.
    Node* regularRect = SpawnSamplePhysicsBox(scene_, worldPosition + Vector3(-2, 0, 0), Vector3(1, 1, 2));

    Node* compoundRootRect = scene_->CreateChild();

    Model* sphereMdl = GetSubsystem<ResourceCache>()->GetResource<Model>("Models/Box.mdl");
    Material* sphereMat = GetSubsystem<ResourceCache>()->GetResource<Material>("Materials/Stone.xml");

    Node* visualNode = compoundRootRect->CreateChild();

    visualNode->SetPosition(Vector3(0, 0, 0.5));
    visualNode->SetScale(Vector3(1, 1, 2));
    StaticModel* sphere1StMdl = visualNode->CreateComponent<StaticModel>();
    sphere1StMdl->SetCastShadows(true);
    sphere1StMdl->SetModel(sphereMdl);
    sphere1StMdl->SetMaterial(sphereMat);

    compoundRootRect->SetWorldPosition(worldPosition + Vector3(2, 0, 0));
    compoundRootRect->CreateComponent<NewtonRigidBody>();
    NewtonCollisionShape_Box* box1 = compoundRootRect->CreateComponent<NewtonCollisionShape_Box>();
	NewtonCollisionShape_Box* box2 = compoundRootRect->CreateComponent<NewtonCollisionShape_Box>();

    //Test different collision parts having different physical properties:
    box1->SetElasticity(1.0f);
    box2->SetElasticity(0.0f);


    box1->SetPositionOffset(Vector3(0, 0, 1));

}

void PhysicsTests::SpawnCompoundedRectTest2(Vector3 worldPosition)
{
    //make 2 1x1x1 physics rectangles. 1 with just one shape and 1 with 2 smaller compounds.

   // Node* regularRect = SpawnSamplePhysicsBox(scene_, worldPosition + Vector3(-2, 0, 0), Vector3(1, 1, 2));
    if(0){
        Node* compoundRootRect = scene_->CreateChild();
        compoundRootRect->SetWorldPosition(worldPosition + Vector3(2, 0, 0));
        compoundRootRect->CreateComponent<NewtonRigidBody>();



        for (int i = 0; i < 2; i++)
        {
            Node* subNode = compoundRootRect->CreateChild();
			NewtonCollisionShape_Box* box = subNode->CreateComponent<NewtonCollisionShape_Box>();

            //box->SetDensity(0.1f);
            subNode->SetPosition(Vector3(5.0f*i, 0, 0));

            Text3D* text = subNode->CreateComponent<Text3D>();
			text->SetText(ea::string("Density: " + ea::to_string(box->GetDensity()) + "\n\n\n\n\n\n\n\n\n"));
            text->SetFont(GetSubsystem<ResourceCache>()->GetResource<Font>("Fonts/Anonymous Pro.ttf"));
            text->SetFaceCameraMode(FC_LOOKAT_XYZ);
            text->SetVerticalAlignment(VA_BOTTOM);



            Model* sphereMdl = GetSubsystem<ResourceCache>()->GetResource<Model>("Models/Box.mdl");
            Material* sphereMat = GetSubsystem<ResourceCache>()->GetResource<Material>("Materials/Stone.xml");



            StaticModel* sphere1StMdl = subNode->CreateComponent<StaticModel>();
            sphere1StMdl->SetCastShadows(true);
            sphere1StMdl->SetModel(sphereMdl);
            sphere1StMdl->SetMaterial(sphereMat);
        }


        compoundRootRect->Rotate(Quaternion(90, Vector3(1, 1, 0)));
    }



    Node* outerNode = scene_->CreateChild();
    outerNode->SetWorldPosition(worldPosition);

    Node* first = nullptr;
    Node* second = nullptr;

    for (int i = 0; i < 2; i++)
    {


        Node* subNode = outerNode->CreateChild();
        NewtonCollisionShape_Box* box = subNode->CreateComponent<NewtonCollisionShape_Box>();
        NewtonRigidBody* rigBody = subNode->CreateComponent<NewtonRigidBody>();


        if (i == 0)
            first = subNode;
        else
            second = subNode;


        subNode->SetPosition(Vector3(5.0f*i, 0, 0) +  Vector3(0,0,5));

        Text3D* text = subNode->CreateComponent<Text3D>();
		text->SetText(ea::string("Density: " + ea::to_string(box->GetDensity()) + "\n\n\n\n\n\n\n\n\n"));
        text->SetFont(GetSubsystem<ResourceCache>()->GetResource<Font>("Fonts/Anonymous Pro.ttf"));
        text->SetFaceCameraMode(FC_LOOKAT_XYZ);
        text->SetVerticalAlignment(VA_BOTTOM);



        Model* sphereMdl = GetSubsystem<ResourceCache>()->GetResource<Model>("Models/Box.mdl");
        Material* sphereMat = GetSubsystem<ResourceCache>()->GetResource<Material>("Materials/Stone.xml");



        StaticModel* sphere1StMdl = subNode->CreateComponent<StaticModel>();
        sphere1StMdl->SetCastShadows(true);
        sphere1StMdl->SetModel(sphereMdl);
        sphere1StMdl->SetMaterial(sphereMat);


    }

    NewtonFullyFixedConstraint* fixedConstriant = first->CreateComponent<NewtonFullyFixedConstraint>();
    fixedConstriant->SetOtherBody(second->GetComponent<NewtonRigidBody>());

    outerNode->Rotate(Quaternion(45, Vector3(0, 1, 0)));
    outerNode->Translate(Vector3(10, 0, 0));
}

void PhysicsTests::SpawnHingeSpringTest(const Vector3 worldPosition, const Quaternion worldOrientation)
{

    Node* baseNode = SpawnSamplePhysicsBox(scene_, worldPosition, Vector3(10,1,1));
    NewtonHingeConstraint* constraint = baseNode->CreateComponent<NewtonHingeConstraint>();
    constraint->SetOwnWorldPosition(worldPosition);
    constraint->SetOtherWorldPosition(worldPosition);
    
    constraint->SetOwnRotation(Quaternion(90, Vector3(0, 1, 0)));
    //constraint->SetOtherWorldRotation(worldOrientation * Quaternion(90, Vector3(0, 1, 0)));

    constraint->SetNoPowerSpringDamper(true);

    // constraint->

    baseNode->SetWorldPosition(worldPosition);
    //baseNode->SetWorldRotation(worldOrientation);

}


void PhysicsTests::SpawnTrialBike(Vector3 worldPosition, Quaternion orientation, bool enableGyroOnWheels)
{
    Node* root = scene_->CreateChild("TrialBike");

    //A (Engine Body)
    Node* A = SpawnSamplePhysicsBox(root, Vector3::ZERO, Vector3(1, 1, 0.5f));

    Node* B = SpawnSamplePhysicsBox(A, Vector3::ZERO + Vector3(-1,0.7,0), Vector3(2, 0.3, 0.5));
    B->RemoveComponent<NewtonRigidBody>();
    B->SetWorldRotation(Quaternion(0, 0, -30));

    Node* C = SpawnSamplePhysicsBox(root, Vector3::ZERO + Vector3(-1, -0.5, 0), Vector3(2, 0.3, 0.5));
    C->SetWorldRotation(Quaternion(0, 0, 0));

    C->GetComponent<NewtonRigidBody>()->SetCollisionOverride(A->GetComponent<NewtonRigidBody>(), false);

    //make back spring.
    NewtonHingeConstraint* hingeConstraint = A->CreateComponent<NewtonHingeConstraint>();
    hingeConstraint->SetOtherBody(C->GetComponent<NewtonRigidBody>());
    hingeConstraint->SetNoPowerSpringDamper(true);
    hingeConstraint->SetNoPowerSpringCoefficient(1000.0f);
    hingeConstraint->SetWorldRotation(Quaternion(90, 0, 90));
    hingeConstraint->SetWorldPosition(A->GetWorldPosition() + Vector3(0,-0.5,0));



    Node* D = SpawnSamplePhysicsBox(A, Vector3::ZERO + Vector3(0.7,0.5,0), Vector3(1,0.5,0.5));
    D->RemoveComponent<NewtonRigidBody>();
    D->SetWorldRotation(Quaternion(0, 0, 45));


    Node* E = SpawnSamplePhysicsBox(root, Vector3::ZERO + Vector3(1.5, 0, 0), Vector3(0.2, 2.5, 0.5));
    E->GetComponent<NewtonRigidBody>()->SetCollisionOverride(A->GetComponent<NewtonRigidBody>(), false);
    E->SetWorldRotation(Quaternion(0, 0, 20));


    NewtonHingeConstraint* hinge = E->CreateComponent<NewtonHingeConstraint>();
    hinge->SetOtherBody(A->GetComponent<NewtonRigidBody>());
    hinge->SetWorldPosition(Vector3::ZERO + Vector3(1.2, 0.8, 0));
    hinge->SetWorldRotation(Quaternion(0,0,-90 + 20));


    Node* F = SpawnSamplePhysicsBox(root, Vector3::ZERO + Vector3(1.5, 0, 0), Vector3(0.2, 2.5, 0.5));
    F->SetWorldRotation(Quaternion(0, 0, 20));
    F->GetComponent<NewtonRigidBody>()->SetCollisionOverride(E->GetComponent<NewtonRigidBody>(), false);
    F->GetComponent<NewtonRigidBody>()->SetCollisionOverride(A->GetComponent<NewtonRigidBody>(), false);

    NewtonSliderConstraint* frontSuspension = F->CreateComponent<NewtonSliderConstraint>();
    frontSuspension->SetOtherBody(E->GetComponent<NewtonRigidBody>());
    frontSuspension->SetWorldRotation(F->GetWorldRotation() * Quaternion(0,0,90));
    frontSuspension->SetEnableSliderSpringDamper(true);
    frontSuspension->SetSliderSpringCoefficient(1000.0f);
    frontSuspension->SetSliderDamperCoefficient(50.0f);
    frontSuspension->SetEnableTwistLimits(true, true);
    frontSuspension->SetTwistLimits(0, 0);
    frontSuspension->SetEnableSliderLimits(true, true);
    frontSuspension->SetSliderLimits(-0.5, 0.5);



    float wheelFriction = 2.0f;

    //backwheel
    Vector3 backWheelOffset = Vector3(-2.0, -0.5, 0);
    Node* backWheel = SpawnSamplePhysicsChamferCylinder(root, Vector3::ZERO + backWheelOffset, 0.8f,0.2f);
    backWheel->SetWorldRotation(Quaternion(90,0,0));
    backWheel->GetComponent<NewtonRigidBody>()->SetCollisionOverride(C->GetComponent<NewtonRigidBody>(), false);
    backWheel->GetComponent<NewtonRigidBody>()->SetUseGyroscopicTorque(enableGyroOnWheels);
    backWheel->GetDerivedComponent<NewtonCollisionShape>()->SetFriction(wheelFriction);


    NewtonHingeConstraint* motor = backWheel->CreateComponent<NewtonHingeConstraint>();
    motor->SetPowerMode(NewtonHingeConstraint::MOTOR);
    motor->SetOtherBody(C->GetComponent<NewtonRigidBody>());
    motor->SetWorldPosition(Vector3::ZERO + backWheelOffset);
    motor->SetWorldRotation(Quaternion(0, 90, 0));
    motor->SetMotorTargetAngularRate(10);
    motor->SetMaxTorque(motor->GetMaxTorque()*0.00125f);





    Vector3 frontWheelOffset = Vector3(1.8, -1, 0);
    Node* frontWheel = SpawnSamplePhysicsChamferCylinder(root, Vector3::ZERO + frontWheelOffset, 0.8f, 0.2f);
    frontWheel->SetWorldRotation(Quaternion(90, 0, 0));
    frontWheel->GetComponent<NewtonRigidBody>()->SetCollisionOverride(E->GetComponent<NewtonRigidBody>(), false);
    frontWheel->GetComponent<NewtonRigidBody>()->SetCollisionOverride(F->GetComponent<NewtonRigidBody>(), false);
    frontWheel->GetComponent<NewtonRigidBody>()->SetUseGyroscopicTorque(enableGyroOnWheels);
    frontWheel->GetDerivedComponent<NewtonCollisionShape>()->SetFriction(wheelFriction);


    NewtonHingeConstraint* frontAxle = frontWheel->CreateComponent<NewtonHingeConstraint>();
    //frontAxle->SetPowerMode(HingeConstraint::MOTOR);
    frontAxle->SetOtherBody(F->GetComponent<NewtonRigidBody>());
    frontAxle->SetWorldPosition(Vector3::ZERO + frontWheelOffset);
    frontAxle->SetWorldRotation(Quaternion(0, 90, 0));
    frontAxle->SetEnableLimits(false);
    //frontAxle->SetMotorTargetAngularRate(10);





    root->SetWorldPosition(worldPosition);
    root->SetWorldRotation(orientation);
}

void PhysicsTests::HandleUpdate(StringHash eventType, VariantMap& eventData)
{
    using namespace Update;

    // Take the frame time step, which is stored as a float
    float timeStep = eventData[P_TIMESTEP].GetFloat();

    // Move the camera, scale movement with time step
    MoveCamera(timeStep);

    UpdatePickPull();



    //move the scene node as a rebuild scene collision test.
    Node* movingNode = scene_->GetChild("MovingSceneNode");


    if (hingeActuatorTest) {
        float angle = Sin(timeAccum*10.0f)*45.0f;

        hingeActuatorTest->SetActuatorTargetAngle(angle);

        timeAccum += timeStep;
    }


    //print forces on force calculation constraints
    for (NewtonConstraint* constraint : forceCalculationConstraints_) {
        //URHO3D_LOGINFO(String(constraint->GetOwnForce()));

    }


   
}

void PhysicsTests::HandlePostRenderUpdate(StringHash eventType, VariantMap& eventData)
{
    // If draw debug mode is enabled, draw physics debug geometry. Use depth test to make the result easier to interpret
    if (drawDebug_) {
        scene_->GetComponent<NewtonPhysicsWorld>()->DrawDebugGeometry(scene_->GetComponent<DebugRenderer>(), false);
        //GetSubsystem<VisualDebugger>()->DrawDebugGeometry(scene_->GetComponent<DebugRenderer>());
    }
}

void PhysicsTests::HandlePhysicsPreStep(StringHash eventType, VariantMap& eventData)
{
    //float timeStep = eventData[PhysicsPreStep::P_TIMESTEP].GetFloat();


    ////rotate the kinamatic body

    ////it is important that the node is rotated by an amount that is scaled by the physics timestep as to match the rigid bodies angular velocity.
    //kinematicNode_->Rotate(Quaternion(10*timeStep, Vector3(0, 1, 0)));
    //kinematicNode_->GetComponent<RigidBody>()->SetAngularVelocity(Vector3(0, 10, 0));
    //kinematicNode_->GetComponent<RigidBody>()->SetWorldTransformToNode();
}

void PhysicsTests::HandlePhysicsPostStep(StringHash eventType, VariantMap& eventData)
{
    float timeStep = eventData[NewtonPhysicsPostStep::P_TIMESTEP].GetFloat();


    //rotate the kinamatic body
    if (kinematicNode_) {

        //it is important that the node is rotated by an amount that is scaled by the physics timestep as to match the rigid bodies angular velocity.
        kinematicNode_->Rotate(Quaternion(0.5, Vector3(0, 1, 0)));
        kinematicNode_->Translate(Vector3(0, 0.01, 0));
        //kinematicNode_->SetWorldTransform(cameraNode_->GetWorldTransform().Translation() + Vector3(0,-5,10), cameraNode_->GetWorldTransform().Rotation(), 1.0f);
        //kinematicNode_->GetComponent<RigidBody>()->SetAngularVelocity(Vector3(0, 10, 0));
    }
}

void PhysicsTests::DecomposePhysicsTree()
{
	ea::vector<RayQueryResult> res;
    Ray ray(cameraNode_->GetWorldPosition(), cameraNode_->GetWorldDirection());
    RayOctreeQuery querry(res, ray);


    scene_->GetComponent<Octree>()->Raycast(querry);

    if (res.size() > 1) {

		ea::vector<Node*> children;
        res[1].node_->GetChildren(children, true);

        //GetSubsystem<VisualDebugger>()->AddOrb(res[1].node_->GetWorldPosition(), 1.0f, Color::RED);

        //for (auto* child : children) {
        //    GSS<VisualDebugger>()->AddOrb(child->GetWorldPosition(), 1.0f, Color(Random(), Random(), Random()));
        //    child->SetParent(scene_);
        //}

        res[1].node_->SetParent(scene_);
    }
}

void PhysicsTests::RecomposePhysicsTree()
{

	ea::vector<Node*> nodes = scene_->GetChildrenWithTag("scaleTestCube", true);

    for (int i = 1; i < nodes.size(); i++) {
        nodes[i]->SetParent(nodes[0]);
    }

}


void PhysicsTests::TransportNode()
{
    RayQueryResult res = GetCameraPickNode();

    if (res.node_) {
		ea::vector<Node*> children;
        if (res.node_->GetName() == "Floor")
            return;

        Node* resolvedNode = res.node_;
        if (!resolvedNode->HasComponent<NewtonRigidBody>())
            resolvedNode = res.node_->GetParent();

        if(resolvedNode != scene_)
            resolvedNode->SetWorldPosition(resolvedNode->GetWorldPosition() + Vector3(Random(), Random()+1.0f, Random())*1.0f);
        //res[1].node_->SetWorldRotation(Quaternion(Random()*360.0f, Random()*360.0f, Random()*360.0f));
    }
}

void PhysicsTests::HandleMouseButtonUp(StringHash eventType, VariantMap& eventData)
{
    ReleasePickTargetOnPhysics();
}

void PhysicsTests::HandleMouseButtonDown(StringHash eventType, VariantMap& eventData)
{

}


void PhysicsTests::HandleCollisionStart(StringHash eventType, VariantMap& eventData)
{
    NewtonRigidBody* bodyA = static_cast<NewtonRigidBody*>(eventData[NewtonPhysicsCollisionStart::P_BODYA].GetPtr());
    NewtonRigidBody* bodyB = static_cast<NewtonRigidBody*>(eventData[NewtonPhysicsCollisionStart::P_BODYB].GetPtr());

    NewtonRigidBodyContactEntry* contactData = static_cast<NewtonRigidBodyContactEntry*>(eventData[NewtonPhysicsCollisionStart::P_CONTACT_DATA].GetPtr());
    //URHO3D_LOGINFO("Collision Start");
    for (int i = 0; i < contactData->numContacts; i++) {
        //GetSubsystem<VisualDebugger>()->AddCross(contactData->contactPositions[i], 0.2f, Color::RED, true);
        
    }


}

void PhysicsTests::HandleCollision(StringHash eventType, VariantMap& eventData)
{
    NewtonRigidBody* bodyA = static_cast<NewtonRigidBody*>(eventData[NewtonPhysicsCollisionStart::P_BODYA].GetPtr());
    NewtonRigidBody* bodyB = static_cast<NewtonRigidBody*>(eventData[NewtonPhysicsCollisionStart::P_BODYB].GetPtr());

    NewtonRigidBodyContactEntry* contactData = static_cast<NewtonRigidBodyContactEntry*>(eventData[NewtonPhysicsCollisionStart::P_CONTACT_DATA].GetPtr());
    //URHO3D_LOGINFO("Collision");
    for (int i = 0; i < contactData->numContacts; i++) {
        //GetSubsystem<VisualDebugger>()->AddCross(contactData->contactPositions[i], 0.2f, Color::GREEN, true);
        
    }
    
}

void PhysicsTests::HandleCollisionEnd(StringHash eventType, VariantMap& eventData)
{
    NewtonRigidBody* bodyA = static_cast<NewtonRigidBody*>(eventData[NewtonPhysicsCollisionStart::P_BODYA].GetPtr());
    NewtonRigidBody* bodyB = static_cast<NewtonRigidBody*>(eventData[NewtonPhysicsCollisionStart::P_BODYB].GetPtr());

	NewtonRigidBodyContactEntry* contactData = static_cast<NewtonRigidBodyContactEntry*>(eventData[NewtonPhysicsCollisionStart::P_CONTACT_DATA].GetPtr());
    //URHO3D_LOGINFO("Collision End\n");
    for (int i = 0; i < contactData->numContacts; i++) {
        //GetSubsystem<VisualDebugger>()->AddCross(contactData->contactPositions[i], 0.2f, Color::BLUE, true);
        
    }
}

RayQueryResult PhysicsTests::GetCameraPickNode()
{
	ea::vector<RayQueryResult> res;
    Ray ray(cameraNode_->GetWorldPosition(), cameraNode_->GetWorldDirection());
    RayOctreeQuery querry(res, ray);
    scene_->GetComponent<Octree>()->Raycast(querry);

    if (res.size() > 1) {
        return res[1];
    }
    return RayQueryResult();
}



void PhysicsTests::CreateScenery(Vector3 worldPosition)
{
    ResourceCache* cache = GetSubsystem<ResourceCache>();

    if (1) {
        // Create a floor object, 1000 x 1000 world units. Adjust position so that the ground is at zero Y
        Node* floorNode = scene_->CreateChild("Floor");
        floorNode->SetPosition(worldPosition - Vector3(0, 0.5f, 0));
        floorNode->SetScale(Vector3(10000.0f, 1.0f, 10000.0f));
        auto* floorObject = floorNode->CreateComponent<StaticModel>();
        floorObject->SetModel(cache->GetResource<Model>("Models/Box.mdl"));
        floorObject->SetMaterial(cache->GetResource<Material>("Materials/StoneTiled.xml"));

        //// Make the floor physical by adding NewtonCollisionShape component. 

        auto* shape = floorNode->CreateComponent<NewtonCollisionShape_Box>();

    }
    else {
        //Create heightmap terrain with collision
        Node* terrainNode = scene_->CreateChild("Terrain");
        terrainNode->SetPosition(worldPosition);
        auto* terrain = terrainNode->CreateComponent<Terrain>();
        terrain->SetPatchSize(64);
        terrain->SetSpacing(Vector3(2.0f, 0.2f, 2.0f)); // Spacing between vertices and vertical resolution of the height map
        terrain->SetSmoothing(true);
        terrain->SetHeightMap(cache->GetResource<Image>("Textures/HeightMap.png"));
        terrain->SetMaterial(cache->GetResource<Material>("Materials/Terrain.xml"));
        // The terrain consists of large triangles, which fits well for occlusion rendering, as a hill can occlude all
        // terrain patches and other objects behind it
        terrain->SetOccluder(true);

        terrainNode->CreateComponent<NewtonCollisionShape_HeightmapTerrain>();
    }



    //ramps
    if (0) {

        for (int i = 0; i < 10; i++) {
      
        Node* ramp = scene_->CreateChild("ramp");
        ramp->SetPosition(worldPosition + Vector3(300*float(i) + 100, 0, 100*(i%2)));
        ramp->SetScale(Vector3(100.0f, 1.0f, 100.0f));
        auto* floorObject = ramp->CreateComponent<StaticModel>();
        floorObject->SetModel(cache->GetResource<Model>("Models/Box.mdl"));
        floorObject->SetMaterial(cache->GetResource<Material>("Materials/StoneTiled.xml"));
        floorObject->SetCastShadows(true);
        ramp->SetWorldRotation(Quaternion(0, 0, 20));

        //// Make the floor physical by adding NewtonCollisionShape component. 
        auto* shape = ramp->CreateComponent<NewtonCollisionShape_Box>();
        }
    }



    float range = 200;
    float objectScale = 100;

    for (int i = 0; i < 0; i++)
    {
        Node* scenePart = scene_->CreateChild("ScenePart" + ea::to_string(i));
        auto* stMdl = scenePart->CreateComponent<StaticModel>();
        stMdl->SetCastShadows(true);
        scenePart->SetPosition(Vector3(Random(-range, range), 0, Random(-range, range)) + worldPosition);
        scenePart->SetRotation(Quaternion(Random(-360, 0), Random(-360, 0), Random(-360, 0)));
        scenePart->SetScale(Vector3(Random(1.0f, objectScale), Random(1.0f, objectScale), Random(1.0f, objectScale)));

        if (i % 2) {
            stMdl->SetModel(cache->GetResource<Model>("Models/Cylinder.mdl"));
            stMdl->SetMaterial(cache->GetResource<Material>("Materials/StoneTiled.xml"));
            NewtonCollisionShape* colShape = scenePart->CreateComponent<NewtonCollisionShape_Cylinder>();
            colShape->SetRotationOffset(Quaternion(0, 0, 90));
        }
        else if (i % 3) {
            stMdl->SetModel(cache->GetResource<Model>("Models/Box.mdl"));
            stMdl->SetMaterial(cache->GetResource<Material>("Materials/StoneTiled.xml"));
            NewtonCollisionShape* colShape = scenePart->CreateComponent<NewtonCollisionShape_Box>();
        }
        else {
            stMdl->SetModel(cache->GetResource<Model>("Models/Sphere.mdl"));
            stMdl->SetMaterial(cache->GetResource<Material>("Materials/StoneTiled.xml"));
            NewtonCollisionShape* colShape = scenePart->CreateComponent<NewtonCollisionShape_Sphere>();
        }
    }






    //// create a moving node for testing scene collision rebuilding.
    //Node* movingSceneNode = scene_->CreateChild("MovingSceneNode");
    //auto* stmdl = movingSceneNode->CreateComponent<StaticModel>();
    //stmdl->SetModel(cache->GetResource<Model>("Models/Box.mdl"));
    //stmdl->SetMaterial(cache->GetResource<Material>("Materials/StoneTiled.xml"));
    //movingSceneNode->CreateComponent<NewtonCollisionShape_Box>();








}

void PhysicsTests::RemovePickNode(bool removeRigidBodyOnly /*= false*/)
{
    RayQueryResult res = GetCameraPickNode();
    if (res.node_) {
        if (removeRigidBodyOnly)
        {
			ea::vector<NewtonRigidBody*> bodies;
            GetRootRigidBodies(bodies, res.node_, false);
            if(bodies.size())
                bodies.back()->GetNode()->Remove();
        }
        else
        {
            res.node_->Remove();
        }
    }
}


void PhysicsTests::SpawnKinematicBodyTest(Vector3 worldPosition, Quaternion worldRotation)
{
    Node* box = SpawnSamplePhysicsBox(scene_, worldPosition, Vector3(10, 1, 10));

    Node* box2 = SpawnSamplePhysicsBox(scene_, worldPosition + Vector3(0, 5,0), Vector3(5, 1, 5));

    box2->SetParent(box);

    box->GetComponent<NewtonRigidBody>()->SetIsKinematic(true);
    box2->GetComponent<NewtonRigidBody>()->SetIsKinematic(true);

    kinematicNode_ = box;


}

void PhysicsTests::SpawnRejointingTest(Vector3 worldPosition)
{
	reJointA = SpawnSamplePhysicsBox(scene_, worldPosition + Vector3(-1.0f, 0, 0), Vector3::ONE);
	reJointB = SpawnSamplePhysicsBox(scene_, worldPosition + Vector3( 1.0f, 0, 0), Vector3::ONE);

	reJointA->SetName("reJointA");
	reJointB->SetName("reJointB");


	NewtonFixedDistanceConstraint* constraint = reJointA->CreateComponent<NewtonFixedDistanceConstraint>();
	
	reJointRoot = scene_->CreateChild("reJointRoot");
	reJointRoot->CreateComponent<NewtonRigidBody>();
	reJointRoot->GetComponent<NewtonRigidBody>()->SetEnabled(false);

	reJointA->SetParent(reJointRoot);
	reJointB->SetParent(reJointRoot);
}




void PhysicsTests::ToggleRejointTest()
{
	URHO3D_LOGINFO("toggling rejoint test");
	bool curEnabled = reJointRoot->GetComponent<NewtonRigidBody>()->IsEnabled();

	reJointA->GetComponent<NewtonRigidBody>()->SetEnabled(curEnabled);
	reJointB->GetComponent<NewtonRigidBody>()->SetEnabled(curEnabled);
	reJointRoot->GetComponent<NewtonRigidBody>()->SetEnabled(!curEnabled);
}

void PhysicsTests::CreatePickTargetNodeOnPhysics()
{
    RayQueryResult res = GetCameraPickNode();
    if (res.node_) {
        if (res.node_->GetName() == "Floor")
            return;

        //get the first root rigid body
		ea::vector<NewtonRigidBody*> rgbodies;
		GetRootRigidBodies(rgbodies, res.node_, false);
		NewtonRigidBody* candidateBody = rgbodies.front();
        if (!candidateBody)
            return;



        //remember the node
        pickPullNode = candidateBody->GetNode();


        //create "PickTarget" on the hit surface, parented to the camera.
        Node* pickTarget = cameraNode_->CreateChild("CameraPullPoint");
        pickTarget->SetWorldPosition(res.position_);
        
        //create/update node that is on the surface of the node.
        if (pickPullNode->GetChild("PickPullSurfaceNode"))
        {
            pickPullNode->GetChild("PickPullSurfaceNode")->SetWorldPosition(res.position_);
        }
        else
        {
            pickPullNode->CreateChild("PickPullSurfaceNode");
            pickPullNode->GetChild("PickPullSurfaceNode")->SetWorldPosition(res.position_);
        }

        pickPullCameraStartOrientation = cameraNode_->GetWorldRotation();


        //make a kinematics joint
		NewtonKinematicsControllerConstraint* constraint = pickPullNode->CreateComponent<NewtonKinematicsControllerConstraint>();
        constraint->SetWorldPosition(pickPullNode->GetChild("PickPullSurfaceNode")->GetWorldPosition());
        constraint->SetWorldRotation(cameraNode_->GetWorldRotation());
        constraint->SetConstrainRotation(false);
        constraint->SetTemporary(true);
    }
}


void PhysicsTests::ReleasePickTargetOnPhysics()
{
    if (pickPullNode)
    {
        pickPullNode->RemoveChild(pickPullNode->GetChild("PickPullSurfaceNode"));
        NewtonRigidBody* rigBody = pickPullNode->GetComponent<NewtonRigidBody>();
        if (rigBody)
        {
            rigBody->ResetForces();
        }
        pickPullNode->RemoveComponent<NewtonKinematicsControllerConstraint>();
        pickPullNode = nullptr;
    }

    cameraNode_->RemoveChild(cameraNode_->GetChild("CameraPullPoint"));
}
void PhysicsTests::UpdatePickPull()
{
    Node* pickTarget = cameraNode_->GetChild("CameraPullPoint");

    if (!pickTarget)
        return;
    if (!pickPullNode)
        return;


    Node* pickSource = pickPullNode->GetChild("PickPullSurfaceNode");

    if (!pickSource)
        return;

    pickPullNode->GetComponent<NewtonKinematicsControllerConstraint>()->SetOtherPosition(pickTarget->GetWorldPosition());
    pickPullNode->GetComponent<NewtonKinematicsControllerConstraint>()->SetOtherRotation(cameraNode_->GetWorldRotation() );

}
