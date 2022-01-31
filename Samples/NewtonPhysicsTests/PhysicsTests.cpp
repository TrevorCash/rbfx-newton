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

#include "Urho3D/Physics/PhysicsWorld.h"
#include <Urho3D/Physics/RigidBody.h>
#include <Urho3D/Resource/ResourceCache.h>
#include <Urho3D/Scene/Scene.h>
#include <Urho3D/UI/Font.h>
#include <Urho3D/UI/Text.h>
#include <Urho3D/UI/UI.h>
#include "Urho3D/UI/Text3D.h"
#include <Urho3D/RmlUI/RmlUI.h>
#include <Urho3D/SystemUI/SystemUI.h>
#include <Urho3D/MLControl/GymClient.h>

#include "Urho3D/Graphics/Terrain.h"
#include "Urho3D/Scene/Node.h"
#include "Urho3D/Scene/ConstructiveSolidGeometry.h"

#include "NewtonPhysicsWorld.h"
#include "NewtonFixedDistanceConstraint.h"
#include "NewtonBallAndSocketConstraint.h"
#include "NewtonKinematicsJoint.h"
#include "NewtonFullyFixedConstraint.h"
#include "NewtonHingeConstraint.h"
#include "NewtonSliderConstraint.h"
#include "NewtonPhysicsEvents.h"
#include "NewtonRigidBody.h"
#include "NewtonGearConstraint.h"
#include <NewtonCollisionShape.h>
#include <NewtonCollisionShapesDerived.h>


#include "PhysicsTests.h"
#include "PhysicsSamplesUtils.h"

#include "GYM_TrialBike.h"
#include "GYM_ATRT.h"
#include "GYM_UniCycle.h"

#include <iostream>

PhysicsTests::PhysicsTests(Context* context) : Application(context),
yaw_(0.0f),
pitch_(0.0f),
touchEnabled_(false),
useMouseMode_(MM_ABSOLUTE),
screenJoystickIndex_(M_MAX_UNSIGNED),
screenJoystickSettingsIndex_(M_MAX_UNSIGNED),
paused_(false)
{}




void PhysicsTests::Start(const ea::vector<ea::string>& args)
{
	PhysicsTests::Start(args);
}

void PhysicsTests::Start()
{
    // Execute base class startup
	if (GetPlatform() == "Android" || GetPlatform() == "iOS")
		// On mobile platform, enable touch by adding a screen joystick
		InitTouchInput();
	else if (GetSubsystem<Input>()->GetNumJoysticks() == 0)
		// On desktop platform, do not detect touch when we already got a joystick
		SubscribeToEvent(E_TOUCHBEGIN, URHO3D_HANDLER(PhysicsTests, HandleTouchBegin));


	// Set custom window Title & Icon
	SetWindowTitleAndIcon();

	// Create console and debug HUD
	CreateConsoleAndDebugHud();

	// Subscribe key down event
	SubscribeToEvent(E_KEYDOWN, URHO3D_HANDLER(PhysicsTests, HandleKeyDown));
	// Subscribe key up event
	SubscribeToEvent(E_KEYUP, URHO3D_HANDLER(PhysicsTests, HandleKeyUp));
	// Subscribe scene update event
	SubscribeToEvent(E_SCENEUPDATE, URHO3D_HANDLER(PhysicsTests, HandleSceneUpdate));

	//Register Newton Physics Lib
	RegisterNewtonPhysicsLibrary(context_);

	context_->RegisterSubsystem<VisualDebugger>();

	context_->RegisterFactory<GYM_TrialBike>();
	context_->RegisterFactory<GYM_ATRT>();
	context_->RegisterFactory<GYM_UniCycle>();
	
	GetSubsystem<Engine>()->SetMinFps(60);


    // Create the scene content
    CreateScene();

    // Create the UI content
    CreateUI();

    // Setup the viewport for displaying the scene
    SetupViewport();

    // Hook up to the frame update and render post-update events
    SubscribeToEvents();

    // Set the mouse mode to use in the sample
    //Sample::InitMouseMode(MM_RELATIVE);
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
    //newtonWorld->SetGravity(Vector3(0, 0, 0));
	
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


	cameraNode_ = new Node(context_);
	auto* camera = cameraNode_->CreateComponent<Camera>();
	camera->SetFarClip(500.0f);
	// Set an initial position for the camera scene node above the floor
	cameraNode_->SetPosition(Vector3(-5.0f, 5.0f, 0));

	//Node* cylinder = SpawnSamplePhysicsCylinder(scene_, Vector3(-5, 2, 0));
	//cylinder->SetName("cylinder1");
	//cylinder->GetComponent<NewtonRigidBody>()->SetCenterOfMassLocalOffset(Vector3(5, 0, 0));

    //SpawnSamplePhysicsCylinder(scene_, Vector3(5, 2, 0), 0.25f,4);
	//SpawnSamplePhysicsBox(scene_, Vector3(5, 5, 0), Vector3(1,1,1));

    //SpawnMaterialsTest(Vector3(0,-25,100));


    //SpawnCompoundedRectTest(Vector3(0, 2, 0));

    //SpawnBallSocketTest(Vector3(50, 10, 0));
    //SpawnHingeActuatorTest(Vector3(52, 10, 0));

    //CreatePyramids(Vector3(0,0,0));


    //SpawnCompound(Vector3(-2, 10 , 10));
    //SpawnConvexHull(Vector3(-2, 3, 10));

	//SpawnATRT(Vector3(5, 5, 0));

	ResetGYMs();	

	//SpawnSegway(Vector3(0,5,0));
	//for(int i = 0; i < 10; i++)
	//	SpawnRobotArm(Vector3(i*10, 5, 0));



    //SpawnKinematicBodyTest(Vector3(0, 0, 0), Quaternion::IDENTITY);


    //SpawnHingeSpringTest(Vector3(0,10,0), Quaternion::IDENTITY);
    //SpawnHingeSpringTest(Vector3(-2, 10, 0), Quaternion(-90, Vector3(0,1,0)));

	//SpawnGearTest(Vector3(0, 0, 0));

    //SpawnCollisionExceptionsTest(Vector3(0, 1, 15));

    //SpawnSliderTest(Vector3(0, 10, 10));
    //SpawnLinearJointedObject(1.0f, Vector3(10 , 2, 10));

    //SpawnNSquaredJointedObject(Vector3(-20, 20, 10));

    //SpawnCompoundedRectTest(Vector3(20, 10, 10));

	//SpawnRejointingTest(Vector3(0, 10, 0));

    ////////create scale test
    //SpawnSceneCompoundTest(Vector3(-20, 10, 20), true);
    //SpawnSceneCompoundTest(Vector3(-20, 10, 30), false); //this was gives newton a non-orthogonal matrix.

    //CreateTowerOfLiar(Vector3(40, 0, 20));

	//SpawnCollisionOffsetTest(Vector3(0, 0, 0));
}
void PhysicsTests::CreateUI()
{
    auto* cache = GetSubsystem<ResourceCache>();
    auto* ui = GetSubsystem<UI>();
	Graphics* graphics = GetSubsystem<Graphics>();

	SharedPtr<BorderImage> crossHair = SharedPtr<BorderImage>(ui->GetRoot()->CreateChild<BorderImage>("sdf"));
	crossHair->SetTexture(GetSubsystem<ResourceCache>()->GetResource<Texture2D>("Textures/UI_Crosshairs.png")); // Set texture

	crossHair->SetBlendMode(BLEND_ADD);
	crossHair->SetSize(64, 64);
	crossHair->SetImageRect(IntRect(128, 0, 64, 64));
	crossHair->SetPosition((graphics->GetWidth() - crossHair->GetWidth()) / 2, (graphics->GetHeight() - crossHair->GetHeight()) / 2);
	crossHair->SetName("crossHair");

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

	if (orbitGYM && gyms.size())
	{
		cameraNode_->LookAt(gyms[0]->orbitNode->GetWorldPosition());
		Vector3 delta = (cameraNode_->GetWorldPosition() - gyms[0]->orbitNode->GetWorldPosition());

		Vector3 curPos = cameraNode_->GetWorldPosition();
		curPos.y_ += 0.1f*timeStep*(curPos.y_ + 5.0 - curPos.y_);
		cameraNode_->SetWorldPosition(curPos);

		cameraNode_->Translate(Vector3(0, 0, 100.0*timeStep*(delta.Length() - 20.0f)));

	}
	else if (!GetSubsystem<Input>()->IsMouseVisible())
	{
		cameraNode_->SetRotation(Quaternion(pitch_, yaw_, 0.0f));
	}


	if (orbitGYM && gyms.size())
	{
		float worldPitch = cameraNode_->GetWorldRotation().EulerAngles().x_;
		float worldYaw = cameraNode_->GetWorldRotation().EulerAngles().y_;
		Quaternion correctedWorldOrientation;
		correctedWorldOrientation.FromEulerAngles(worldPitch, worldYaw, 0);
		cameraNode_->SetWorldRotation(correctedWorldOrientation);
	}	
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

	hoverNode = GetCameraPickNode().node_;


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

	if (input->GetKeyPress(KEY_5))
	{
		scene_->GetChild("cylinder1")->SetWorldPosition(Vector3(10, 10, 0));
	}


}



void PhysicsTests::SpawnSceneCompoundTest(const Vector3& worldPos, bool oneBody)
{
	Node* root = scene_->CreateChild("SpawnSceneCompoundTestRoot");
    const int breadth = 2;
	const int levelCount = 10;
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

    const int numSpheres = 25;

    ea::vector<Node*> nodes;
    //make lots of spheres
    for (int i = 0; i < numSpheres; i++)
    {
        Node* node = SpawnSamplePhysicsSphere(scene_, worldPosition  + Vector3(0,dist*0.5f,0) - Quaternion(Random()*360.0f, 
			Random()*360.0f, Random()*360.0f) * (Vector3::FORWARD*dist));
      
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

    const int numSpheres = 100;

	ea::vector<Node*> nodes;
    //make lots of spheres
    for (int i = 0; i < numSpheres; i++)
    {
		nodes.push_back(SpawnSamplePhysicsSphere(scene_, worldPosition + Vector3(0, i*dist, 0), dist*0.5f));

        if (i > 0) {
            NewtonFixedDistanceConstraint* constraint = nodes[i - 1]->CreateComponent<NewtonFixedDistanceConstraint>();
            constraint->SetOtherBody(nodes[i]->GetComponent<NewtonRigidBody>());
            constraint->SetWorldPosition(worldPosition + Vector3(0, i*dist, 0) - Vector3(0, dist, 0)*0.5f);

			
            
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


    //constraint->SetEnableTwistLimits(true, true);
    //constraint->SetTwistLimits(-180, 180);




    //constraint->SetEnableSliderSpringDamper(true);
    //constraint->SetEnableTwistSpringDamper(true);

}

void PhysicsTests::SpawnGearTest(Vector3 worldPosition)
{
	Node* box1 = SpawnSamplePhysicsBox(scene_, worldPosition, Vector3(1, 1, 1));
	Node* box2 = SpawnSamplePhysicsBox(scene_, worldPosition + Vector3(5, 0, 0), Vector3(1, 1, 1));

	NewtonGearConstraint* constraint = box1->CreateComponent<NewtonGearConstraint>();

	constraint->SetOtherBody(box2->GetComponent<NewtonRigidBody>());

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

        node->SetScale(0.5f);
        node->GetComponent<NewtonRigidBody>()->SetLinearVelocity(cameraNode_->GetWorldDirection() * 10.0f);
        //node->GetComponent<NewtonRigidBody>()->SetContinuousCollision(false);
        //node->GetComponent<RigidBody>()->SetLinearDamping(0.01f);
        //node->GetComponent<RigidBody>()->SetMassScale(Random(1.0f, 10.0f));
        node->GetComponent<NewtonRigidBody>()->SetGenerateContacts(false);

    }

}

void PhysicsTests::SpawnCompoundedRectTest(Vector3 worldPosition)
{
    //make 2 1x1x1 physics boxes side by side in one rigid body with 1 collision shape rotated

	{
		Node* root1 = scene_->CreateChild();
		root1->SetWorldPosition(worldPosition + Vector3(0, 0, 0));
		root1->SetScale(Vector3(1, 1, 1));

		Model* sphereMdl = GetSubsystem<ResourceCache>()->GetResource<Model>("Models/Box.mdl");
		Material* sphereMat = GetSubsystem<ResourceCache>()->GetResource<Material>("Materials/Stone.xml");

		Node* visualNode = root1->CreateChild();

		visualNode->SetPosition(Vector3(0, 0, 0.5));
		visualNode->SetScale(Vector3(1, 1, 2));
		StaticModel* sphere1StMdl = visualNode->CreateComponent<StaticModel>();
		sphere1StMdl->SetCastShadows(true);
		sphere1StMdl->SetModel(sphereMdl);
		sphere1StMdl->SetMaterial(sphereMat);


		root1->CreateComponent<NewtonRigidBody>();
		NewtonCollisionShape_Box* box1 = root1->CreateComponent<NewtonCollisionShape_Box>();
		NewtonCollisionShape_Box* box2 = root1->CreateComponent<NewtonCollisionShape_Box>();

		//Test different collision parts having different physical properties:
		box1->SetElasticity(1.0f);
		box2->SetElasticity(0.0f);

		box2->SetRotationOffset(Quaternion(30, Vector3::RIGHT));

		box1->SetPositionOffset(Vector3(0, 0, 1));
	}

	//make 2 1x1x1 physics boxes side by side in one rigid body with 1 shape rotated using child nodes
	{
		Node* root2 = scene_->CreateChild();
		root2->SetWorldPosition(worldPosition + Vector3(2, 0, 0));
		root2->SetScale(Vector3(1, 1, 1));

		Node* box1Node = root2->CreateChild();
		Node* box2Node = root2->CreateChild();
		box2Node->SetPosition(Vector3(0, 0, -1));
		box2Node->Rotate(Quaternion(30, Vector3(1, 0, 0)));
		box2Node->SetScale(Vector3(1, 1, 1.5));
		NewtonCollisionShape_Box* box1 = box1Node->CreateComponent<NewtonCollisionShape_Box>();
		NewtonCollisionShape_Box* box2 = box2Node->CreateComponent<NewtonCollisionShape_Box>();


		Model* mdl = GetSubsystem<ResourceCache>()->GetResource<Model>("Models/Box.mdl");
		Material* mat = GetSubsystem<ResourceCache>()->GetResource<Material>("Materials/Stone.xml");

		Node* visualNode1 = box1Node->CreateChild();
		Node* visualNode2 = box2Node->CreateChild();


		StaticModel* stmdl = visualNode1->CreateComponent<StaticModel>();
		stmdl->SetCastShadows(true);
		stmdl->SetModel(mdl);
		stmdl->SetMaterial(mat);

    	stmdl = visualNode2->CreateComponent<StaticModel>();
		stmdl->SetCastShadows(true);
		stmdl->SetModel(mdl);
		stmdl->SetMaterial(mat);



		root2->CreateComponent<NewtonRigidBody>();

		//Test different collision parts having different physical properties:
		box1->SetElasticity(1.0f);
		box2->SetElasticity(0.0f);

	}


}

void PhysicsTests::SpawnCompoundedRectTest2(Vector3 worldPosition)
{
    //make 2 1x1x1 physics rectangles. 1 with just one shape and 1 with 2 smaller compounds.

   // Node* regularRect = SpawnSamplePhysicsBox(scene_, worldPosition + Vector3(-2, 0, 0), Vector3(1, 1, 2));
    if(1){
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


void PhysicsTests::ResetGYMs()
{
	URHO3D_LOGINFO("ResetGYMs");
	GymClient* gymCli = context_->GetSubsystem<GymClient>();
	

	while (gyms.size() > gymCli->numGYMS)
	{
		gyms.back()->TearDown();
		gyms.pop_back();
	}
	while (gyms.size() < gymCli->numGYMS)
	{
		gyms.push_back(context_->CreateObject<GYM_TrialBike>());
		//gyms.push_back(context_->CreateObject<GYM_ATRT>());
		//gyms.push_back(context_->CreateObject<GYM_UniCycle>());
	}


	//int sqrt = Sqrt(gymCli->numGYMS);

	int i = 0; 

	for (int y = 0; y <= gymCli->numGYMS; y++)
	{
		if (i >= gyms.size()) continue;
		gyms[i]->worldPos = Vector3(0, 3.5, y*5);
		i++;

	}
	

	for (int i = 0; i < gymCli->numGYMS; i++)
	{
		gyms[i]->scene_ = scene_;
		gyms[i]->Reset();
		gyms[i]->PostReset();
	}



	context_->GetSubsystem<GymClient>()->SetGYMSpec(gyms[0]->actionVec.size(), gyms[0]->stateVec.size());

	EvalOrbitGym();
	URHO3D_LOGINFO("GYM Reset Finished");


}
void PhysicsTests::SpawnATRT(Vector3 worldPosition)
{
	Node* root = scene_->CreateChild("ATRT");
	root->AddTag("ATRT");

	//Body
	Node* Body = SpawnSamplePhysicsBox(root, Vector3::ZERO, Vector3(1, 1, 1));


	//LEFT LEG
	Node* HIP_LEFT = SpawnSamplePhysicsCylinder(root, Vector3(0.5, -0.5, -0.5), 0.5, 0.25);
	HIP_LEFT->Rotate(Quaternion(90, Vector3(1, 0, 0)));

	NewtonHingeConstraint* HIPBODYJOINT_LEFT = Body->CreateComponent<NewtonHingeConstraint>();
	HIPBODYJOINT_LEFT->SetRotation(Quaternion(90, Vector3(0, 1, 0)));
	HIPBODYJOINT_LEFT->SetPosition(Vector3(0.5, -0.5, -0.5));
	HIPBODYJOINT_LEFT->SetOtherBody(HIP_LEFT->GetComponent<NewtonRigidBody>());

	NewtonHingeConstraint* HIPBODYJOINT_LEFT_R = Body->CreateComponent<NewtonHingeConstraint>();
	HIPBODYJOINT_LEFT_R->SetRotation(Quaternion(90, Vector3(0, 1, 0)));
	HIPBODYJOINT_LEFT_R->SetPosition(Vector3(0.5, -0.5, -0.5));
	HIPBODYJOINT_LEFT_R->SetOtherBody(HIP_LEFT->GetComponent<NewtonRigidBody>());



	Node* KNEE_LEFT = SpawnSamplePhysicsCylinder(HIP_LEFT, Vector3(1.0, -1.5, -0.5), 0.3, 0.25);
	KNEE_LEFT->RemoveComponent<NewtonRigidBody>();

	Node* KNEE2_LEFT = SpawnSamplePhysicsCylinder(root, Vector3(1.0, -1.5, -0.75), 0.3, 0.25);
	KNEE2_LEFT->Rotate(Quaternion(90, Vector3(1, 0, 0)));

	NewtonHingeConstraint* KNEEJOINT_LEFT = HIP_LEFT->CreateComponent<NewtonHingeConstraint>();
	KNEEJOINT_LEFT->SetRotation(Quaternion(90, Vector3(0, 0, 1)));
	KNEEJOINT_LEFT->SetWorldPosition(KNEE_LEFT->GetWorldPosition());
	KNEEJOINT_LEFT->SetOtherBody(KNEE2_LEFT->GetComponent<NewtonRigidBody>());

	NewtonHingeConstraint* KNEEJOINT_LEFT_R = HIP_LEFT->CreateComponent<NewtonHingeConstraint>();
	KNEEJOINT_LEFT_R->SetRotation(Quaternion(90, Vector3(0, 0, 1)));
	KNEEJOINT_LEFT_R->SetWorldPosition(KNEE_LEFT->GetWorldPosition());
	KNEEJOINT_LEFT_R->SetOtherBody(KNEE2_LEFT->GetComponent<NewtonRigidBody>());







	Node* KNEE3_LEFT = SpawnSamplePhysicsCylinder(KNEE2_LEFT, Vector3(1.0, -2.5, -0.5), 0.3, 0.25);
	KNEE3_LEFT->RemoveComponent<NewtonRigidBody>();

	Node* KNEE4_LEFT = SpawnSamplePhysicsCylinder(root, Vector3(1.0, -2.5, -0.75), 0.3, 0.25);
	KNEE4_LEFT->Rotate(Quaternion(90, Vector3(1, 0, 0)));

	NewtonHingeConstraint* KNEEJOINT2_LEFT = KNEE2_LEFT->CreateComponent<NewtonHingeConstraint>();
	KNEEJOINT2_LEFT->SetRotation(Quaternion(90, Vector3(0, 0, 1)));
	KNEEJOINT2_LEFT->SetWorldPosition(KNEE3_LEFT->GetWorldPosition());
	KNEEJOINT2_LEFT->SetOtherBody(KNEE4_LEFT->GetComponent<NewtonRigidBody>());

	NewtonHingeConstraint* KNEEJOINT2_LEFT_R = KNEE2_LEFT->CreateComponent<NewtonHingeConstraint>();
	KNEEJOINT2_LEFT_R->SetRotation(Quaternion(90, Vector3(0, 0, 1)));
	KNEEJOINT2_LEFT_R->SetWorldPosition(KNEE3_LEFT->GetWorldPosition());
	KNEEJOINT2_LEFT_R->SetOtherBody(KNEE4_LEFT->GetComponent<NewtonRigidBody>());


	Node* FOOT_LEFT = SpawnSamplePhysicsCylinder(KNEE4_LEFT, Vector3(0, -3, -0.5), 0.2, 0.25);
	FOOT_LEFT->RemoveComponent<NewtonRigidBody>();

	//RIGHT LEG
	Node* HIP_RIGHT = SpawnSamplePhysicsCylinder(root, Vector3(0.5, -0.5, 0.5), 0.5, 0.25);
	HIP_RIGHT->Rotate(Quaternion(90, Vector3(1, 0, 0)));

	NewtonHingeConstraint* HIPBODYJOINT_RIGHT = Body->CreateComponent<NewtonHingeConstraint>();
	HIPBODYJOINT_RIGHT->SetRotation(Quaternion(90, Vector3(0, 1, 0)));
	HIPBODYJOINT_RIGHT->SetPosition(Vector3(0.5, -0.5, 0.5));
	HIPBODYJOINT_RIGHT->SetOtherBody(HIP_RIGHT->GetComponent<NewtonRigidBody>());

	NewtonHingeConstraint* HIPBODYJOINT_RIGHT_R = Body->CreateComponent<NewtonHingeConstraint>();
	HIPBODYJOINT_RIGHT_R->SetRotation(Quaternion(90, Vector3(0, 1, 0)));
	HIPBODYJOINT_RIGHT_R->SetPosition(Vector3(0.5, -0.5, 0.5));
	HIPBODYJOINT_RIGHT_R->SetOtherBody(HIP_RIGHT->GetComponent<NewtonRigidBody>());


	Node* KNEE_RIGHT = SpawnSamplePhysicsCylinder(HIP_RIGHT, Vector3(1.0, -1.5, 0.75), 0.3, 0.25);
	KNEE_RIGHT->RemoveComponent<NewtonRigidBody>();

	Node* KNEE2_RIGHT = SpawnSamplePhysicsCylinder(root, Vector3(1.0, -1.5, 0.5), 0.3, 0.25);
	KNEE2_RIGHT->Rotate(Quaternion(90, Vector3(1, 0, 0)));

	NewtonHingeConstraint* KNEEJOINT_RIGHT = HIP_RIGHT->CreateComponent<NewtonHingeConstraint>();
	KNEEJOINT_RIGHT->SetRotation(Quaternion(90, Vector3(0, 0, 1)));
	KNEEJOINT_RIGHT->SetWorldPosition(KNEE_RIGHT->GetWorldPosition());
	KNEEJOINT_RIGHT->SetOtherBody(KNEE2_RIGHT->GetComponent<NewtonRigidBody>());

	NewtonHingeConstraint* KNEEJOINT_RIGHT_R = HIP_RIGHT->CreateComponent<NewtonHingeConstraint>();
	KNEEJOINT_RIGHT_R->SetRotation(Quaternion(90, Vector3(0, 0, 1)));
	KNEEJOINT_RIGHT_R->SetWorldPosition(KNEE_RIGHT->GetWorldPosition());
	KNEEJOINT_RIGHT_R->SetOtherBody(KNEE2_RIGHT->GetComponent<NewtonRigidBody>());


	Node* KNEE3_RIGHT = SpawnSamplePhysicsCylinder(KNEE2_RIGHT, Vector3(1.0, -2.5, 0.75), 0.3, 0.25);
	KNEE3_RIGHT->RemoveComponent<NewtonRigidBody>();

	Node* KNEE4_RIGHT = SpawnSamplePhysicsCylinder(root, Vector3(1.0, -2.5, 0.5), 0.3, 0.25);
	KNEE4_RIGHT->Rotate(Quaternion(90, Vector3(1, 0, 0)));

	NewtonHingeConstraint* KNEEJOINT2_RIGHT = KNEE2_RIGHT->CreateComponent<NewtonHingeConstraint>();
	KNEEJOINT2_RIGHT->SetRotation(Quaternion(90, Vector3(0, 0, 1)));
	KNEEJOINT2_RIGHT->SetWorldPosition(KNEE3_RIGHT->GetWorldPosition());
	KNEEJOINT2_RIGHT->SetOtherBody(KNEE4_RIGHT->GetComponent<NewtonRigidBody>());

	NewtonHingeConstraint* KNEEJOINT2_RIGHT_R = KNEE2_RIGHT->CreateComponent<NewtonHingeConstraint>();
	KNEEJOINT2_RIGHT_R->SetRotation(Quaternion(90, Vector3(0, 0, 1)));
	KNEEJOINT2_RIGHT_R->SetWorldPosition(KNEE3_RIGHT->GetWorldPosition());
	KNEEJOINT2_RIGHT_R->SetOtherBody(KNEE4_RIGHT->GetComponent<NewtonRigidBody>());

	Node* FOOT_RIGHT = SpawnSamplePhysicsCylinder(KNEE4_RIGHT, Vector3(0, -3, 0.5), 0.2, 0.25);
	FOOT_RIGHT->RemoveComponent<NewtonRigidBody>();


	NewtonHingeConstraint::PoweredMode powerMode = NewtonHingeConstraint::MOTOR_TORQUE;
	HIPBODYJOINT_LEFT->SetPowerMode(powerMode);
	HIPBODYJOINT_RIGHT->SetPowerMode(powerMode);
	KNEEJOINT_LEFT->SetPowerMode(powerMode);
	KNEEJOINT_RIGHT->SetPowerMode(powerMode);
	KNEEJOINT2_LEFT->SetPowerMode(powerMode);
	KNEEJOINT2_RIGHT->SetPowerMode(powerMode);

	//ea::vector<NewtonHingeConstraint*> motors;
	//motors.push_back(HIPBODYJOINT_LEFT);
	//motors.push_back(HIPBODYJOINT_RIGHT);
	//motors.push_back(KNEEJOINT_LEFT);
	//motors.push_back(KNEEJOINT_RIGHT);
	//motors.push_back(KNEEJOINT2_LEFT);
	//motors.push_back(KNEEJOINT2_RIGHT);
	//GymMotors.push_back(motors);

	//GymBodies.push_back(Body);

	root->SetWorldPosition(worldPosition);
	root->Rotate(Quaternion(RandomNormal(0, 45), Vector3(0, 0, 1)));

	Body->GetComponent<NewtonRigidBody>()->SetLinearVelocityHard(Vector3(0, 0.1, 0));
}



void PhysicsTests::SpawnSegway(Vector3 worldPosition)
{
    Node* root = scene_->CreateChild("SegWay");

    Node* Body = SpawnSamplePhysicsBox(root, Vector3::ZERO, Vector3(0.5, 4, 0.5));

    Node* Wheel = SpawnSamplePhysicsCylinder(root, Vector3(0, -3, 0), 1.0, 0.125);
    Wheel->Rotate(Quaternion(90, Vector3(1, 0, 0)));

    NewtonHingeConstraint* motor = Body->CreateComponent<NewtonHingeConstraint>();
    motor->SetRotation(Quaternion(90, Vector3(0, 1, 0)));
    motor->SetPosition(Vector3(0, -3, 0));
    motor->SetEnableLimits(false);
    motor->SetPowerMode(NewtonHingeConstraint::MOTOR_TORQUE);
    motor->SetOtherBody(Wheel->GetComponent<NewtonRigidBody>());
	//motor->SetMotorTorque(1);
	//motor->SetMotorMaxAngularRate(99999.0f);

	//topwiegth
	Node* top = SpawnSamplePhysicsCylinder(root, Vector3(0, 3, 0), 1);

	NewtonHingeConstraint* topMotor = Body->CreateComponent<NewtonHingeConstraint>();

	topMotor->SetPosition(Vector3(0, 3, 0));
	topMotor->SetRotation(Quaternion(90, Vector3(0, 0, 1)));
	topMotor->SetEnableLimits(false);
	topMotor->SetPowerMode(NewtonHingeConstraint::MOTOR_TORQUE);
	topMotor->SetOtherBody(top->GetComponent<NewtonRigidBody>());
	//topMotor->SetMotorTorque(1);


    root->SetWorldPosition(worldPosition);
}


void PhysicsTests::SpawnRobotArm(Vector3 worldPosition)
{
	Node* root = scene_->CreateChild("RobotArm");

	Node* base = SpawnSamplePhysicsCylinder(root, Vector3::ZERO, 2,0.5);
	base->GetComponent<NewtonCollisionShape_Cylinder>()->SetDensity(100);

	Node* base2 = SpawnSamplePhysicsCylinder(root, Vector3(0,1,0), 1, 1);

	NewtonHingeConstraint* motor1 = base->CreateComponent<NewtonHingeConstraint>();
	motor1->SetOtherBody(base2->GetComponent<NewtonRigidBody>());
	motor1->SetRotation(Quaternion(90, Vector3(0, 0, 1)));
	motor1->SetEnableLimits(false);


	Node* arm1 = SpawnSamplePhysicsBox(root, Vector3(0, 2, 0), Vector3(1, 3, 1));
	NewtonHingeConstraint* motor2 = base2->CreateComponent<NewtonHingeConstraint>();
	motor2->SetOtherBody(arm1->GetComponent<NewtonRigidBody>());


	Node* arm2 = SpawnSamplePhysicsBox(root, Vector3(0, 4, 0), Vector3(1, 3, 1));
	NewtonHingeConstraint* motor3 = arm1->CreateComponent<NewtonHingeConstraint>();
	motor3->SetOtherBody(arm2->GetComponent<NewtonRigidBody>());
	motor3->SetPosition(Vector3(0,1,0));

	Node* arm3 = SpawnSamplePhysicsBox(root, Vector3(0, 6, 0), Vector3(1, 3, 1));
	NewtonHingeConstraint* motor4 = arm2->CreateComponent<NewtonHingeConstraint>();
	motor4->SetOtherBody(arm3->GetComponent<NewtonRigidBody>());
	motor4->SetPosition(Vector3(0, 1, 0));

	Node* wrist = SpawnSamplePhysicsCylinder(root, Vector3(0, 7.5, 0), 0.5, 0.5);
	NewtonHingeConstraint* motor5 = arm3->CreateComponent<NewtonHingeConstraint>();
	motor5->SetOtherBody(wrist->GetComponent<NewtonRigidBody>());
	motor5->SetPosition(Vector3(0, 1, 0));
	motor5->SetRotation(Quaternion(90, Vector3(0, 0, 1)));

	Node* arm4 = SpawnSamplePhysicsBox(root, Vector3(0, 8.5, 0), Vector3(0.5, 1.5, 0.5));
	NewtonHingeConstraint* motor6 = wrist->CreateComponent<NewtonHingeConstraint>();
	motor6->SetOtherBody(arm4->GetComponent<NewtonRigidBody>());
	motor6->SetPosition(Vector3(0, 0, 0));



	root->SetWorldPosition(worldPosition);
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


	NewtonHingeConstraint* constraint = reJointA->CreateComponent<NewtonHingeConstraint>();
	constraint->SetNoPowerSpringDamper(true);
	constraint->SetRotation(Quaternion(90, Vector3(0, 1, 0)));


	reJointRoot = scene_->CreateChild("reJointRoot");
	reJointRoot->CreateComponent<NewtonRigidBody>();
	reJointRoot->GetComponent<NewtonRigidBody>()->SetEnabled(false);

	reJointA->SetParent(reJointRoot);
	reJointB->SetParent(reJointRoot);
}






void PhysicsTests::SpawnCollisionOffsetTest(Vector3 worldPosition)
{
	
	Node* boxA = SpawnSamplePhysicsBox(scene_, worldPosition + Vector3(-1,0,0), Vector3::ONE);
	Node* boxB = SpawnSamplePhysicsBox(scene_, worldPosition + Vector3( 1,0,0 ), Vector3::ONE);

	//alter boxB so its collision is offset by a large amount.
	boxB->GetDerivedComponent<NewtonCollisionShape>()->SetPositionOffset(Vector3(0, 10, 10));

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

	GymClient* GymCli = context_->GetSubsystem<GymClient>();
	if (GymCli->resetPending)
	{
		ResetGYMs();

		GymCli->resetPending = false;
	}
	else
	{
		for (int i = 0; i < gyms.size(); i++)
		{
			//get actions from gymclient
			gyms[i]->actionVec = GymCli->actionSets[i];


			gyms[i]->Update(timeStep);
		}
	}

	//gather response data and copy to GymCli
	for (int i = 0; i < gyms.size(); i++)
	{
		GymCli->states[i] = gyms[i]->stateVec;
		GymCli->rewards[i] = gyms[i]->reward;
		GymCli->ends[i] = gyms[i]->end;
	}
   
}

void PhysicsTests::HandlePostRenderUpdate(StringHash eventType, VariantMap& eventData)
{
	// Take the frame time step, which is stored as a float
	float timeStep = eventData[Update::P_TIMESTEP].GetFloat();
if (drawDebug_) {
	bool doFrSim = ui::Button("ForwardSim", ImVec2(100, 20));
	bool openGYM = ui::Button("Connect to GYM Server", ImVec2(100, 20));
	bool resetGYM = ui::Button("Reset", ImVec2(100, 20));
	bool saveNewtonPressed = ui::Button("Save Newton Scene", ImVec2(100, 20));

	bool orbitGYMPressed = ui::Checkbox("Orbit GYM", &orbitGYM);


	ui::Text("NumGYMS: %d", context_->GetSubsystem<GymClient>()->numGYMS);
	if (doFrSim)
	{
		context_->GetSubsystem<Engine>()->FrameSkip(10, 1 / 60.0f);
	}

	if (openGYM)
	{

		context_->GetSubsystem<GymClient>()->Connect();
	}
	if (orbitGYMPressed || resetGYM) {

		EvalOrbitGym();
	}
	if (resetGYM)
	{
		context_->GetSubsystem<GymClient>()->resetPending = true;
	}
	if(saveNewtonPressed)
	{
		ndLoadSave loadScene;

		ndWorld* newtonWorld = scene_->GetComponent<NewtonPhysicsWorld>()->GetNewtonWorld();
		ndWordSettings setting;

		loadScene.SaveScene("compoundTransforms", newtonWorld, &setting);

	}


	if (gyms.size())
	{

		gyms[0]->DrawUIStats(timeStep);
	}



	if(hoverNode )
	{
		ea::vector<NewtonRigidBody*> rigBodies;
		GetRootRigidBodies(rigBodies, hoverNode, true);
		if (rigBodies.size())
		{

			ui::Begin("NewtonBody Info");


			Matrix3 inertia = rigBodies[0]->GetMassMatrix();
			ui::Text("Inertia Matrix:");
			ui::Text("%f,%f,%f\r\n%f,%f,%f\r\n%f,%f,%f\r\n", 
				inertia.m00_, inertia.m01_, inertia.m02_,
				inertia.m10_, inertia.m11_, inertia.m12_,
				inertia.m20_, inertia.m21_, inertia.m22_);

			ui::Text("Mass: %f", rigBodies[0]->GetEffectiveMass());
			Vector3 netForce = rigBodies[0]->GetNetWorldForce();
			ui::Text("Net Force (NoGrav)(World): %f,%f,%f", netForce.x_, netForce.y_, netForce.z_);
			Vector3 netTorque = rigBodies[0]->GetNetWorldTorque();
			ui::Text("Net Torque (NoGrav)(World): %f,%f,%f", netTorque.x_, netTorque.y_, netTorque.z_);

			ui::End();
		}
	}






	scene_->GetComponent<NewtonPhysicsWorld>()->DrawDebugGeometry(scene_->GetComponent<DebugRenderer>(), false);
	GetSubsystem<VisualDebugger>()->DrawDebugGeometry(scene_->GetComponent<DebugRenderer>());

	for (int i = 0; i < gyms.size(); i++)
	{
		gyms[i]->DrawDebugGeometry(scene_->GetComponent<DebugRenderer>());
	}
}

	

}

void PhysicsTests::EvalOrbitGym()
{

}

void PhysicsTests::HandlePhysicsPreStep(StringHash eventType, VariantMap& eventData)
{



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




}

void PhysicsTests::HandleCollision(StringHash eventType, VariantMap& eventData)
{
    NewtonRigidBody* bodyA = static_cast<NewtonRigidBody*>(eventData[NewtonPhysicsCollisionStart::P_BODYA].GetPtr());
    NewtonRigidBody* bodyB = static_cast<NewtonRigidBody*>(eventData[NewtonPhysicsCollisionStart::P_BODYB].GetPtr());

    
}

void PhysicsTests::HandleCollisionEnd(StringHash eventType, VariantMap& eventData)
{
    NewtonRigidBody* bodyA = static_cast<NewtonRigidBody*>(eventData[NewtonPhysicsCollisionStart::P_BODYA].GetPtr());
    NewtonRigidBody* bodyB = static_cast<NewtonRigidBody*>(eventData[NewtonPhysicsCollisionStart::P_BODYB].GetPtr());

	
    
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
        floorNode->SetScale(Vector3(1000.0f, 1.0f, 1000.0f));
        auto* floorObject = floorNode->CreateComponent<StaticModel>();
        floorObject->SetModel(cache->GetResource<Model>("Models/Box.mdl"));
		SharedPtr<Material> floorMat = cache->GetResource<Material>("Materials/Brick.xml")->Clone();

		floorMat->SetUVTransform(Vector2(), 0, 500);
		floorMat->SetShaderParameter("MatDiffColor", Vector4(1.0, 1.0, 1.0, 0.1f));
        floorObject->SetMaterial(floorMat);
		//floorNode->SetRotation(Quaternion(10, Vector3(1, 0, 0)));

        auto* shape = floorNode->CreateComponent<NewtonCollisionShape_Box>();

		if(0){
			Node* boxNode = floorNode->CreateChild("StaticBox");
			boxNode->SetRotation(Quaternion(10, Vector3(1, 0, 0)));
			boxNode->SetScale(Vector3(1, 1, 1));

			boxNode->SetPosition(Vector3(0, 10, 0));
			auto* boxShape = boxNode->CreateComponent<NewtonCollisionShape_Box>();

			auto* boxObject = boxNode->CreateComponent<StaticModel>();
			boxObject->SetModel(cache->GetResource<Model>("Models/Box.mdl"));
			SharedPtr<Material> boxMat = cache->GetResource<Material>("Materials/Metal.xml")->Clone();

			boxMat->SetUVTransform(Vector2(), 0, 10);
			boxMat->SetShaderParameter("MatDiffColor", Vector4(1.0, 1.0, 1.0, 0.1f));
			boxObject->SetMaterial(boxMat);
		}


		SharedPtr<CSGManipulator> csg = context_->CreateObject<CSGManipulator>();
        Node* csgRes = scene_->CreateChild("CSGRES");
        auto* resMdl = csgRes->CreateComponent<StaticModel>();
        resMdl->SetModel(cache->GetResource<Model>("Models/Box.mdl"));
    	csg->SetResultNode(csgRes);

        Node* boxNode = scene_->CreateChild("StaticBox");
        SharedPtr<Material> boxMat = cache->GetResource<Material>("Materials/Brick.xml")->Clone();
        auto* boxMdl = boxNode->CreateComponent<StaticModel>();
        boxMdl->SetModel(cache->GetResource<Model>("Models/Box.mdl"));
        boxMdl->SetMaterial(boxMat);
        boxNode->SetScale(1);

        Node* boxNode2 = scene_->CreateChild("StaticBox");
        boxMdl = boxNode2->CreateComponent<StaticModel>();
        boxMdl->SetModel(cache->GetResource<Model>("Models/Box.mdl"));
        boxMdl->SetMaterial(boxMat);
        boxNode2->SetPosition(Vector3(0.5, 0.5, 0.5));
        //boxNode2->SetRotation(Quaternion(30, Vector3(1, 0, 0)));
        //boxNode2->SetScale(5);
        csg->Union(boxNode);
        csg->Subtract(boxNode2);

        csg->BakeSingle();

        csgRes->SetPosition(Vector3(10, 0, 0));
        resMdl->SetMaterial(boxMat);

        csgRes->CreateComponent<NewtonCollisionShape_TreeCollision>();
    }
    else {
        //Create heightmap terrain with collision
        Node* terrainNode = scene_->CreateChild("Terrain");
        terrainNode->SetPosition(worldPosition - Vector3(0,1, 0));
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


void PhysicsTests::ToggleRejointTest()
{
	URHO3D_LOGINFO("toggling rejoint test");
	bool curEnabled = reJointRoot->GetComponent<NewtonRigidBody>()->IsEnabled();

	reJointA->GetComponent<NewtonRigidBody>()->SetEnabled(curEnabled);
	reJointB->GetComponent<NewtonRigidBody>()->SetEnabled(curEnabled);
	reJointRoot->GetComponent<NewtonRigidBody>()->SetEnabled(!curEnabled);

	if (!curEnabled)
	{
		reJointRoot->GetComponent<NewtonRigidBody>()->ApplyMomentumFromRigidBodyChildren(true);
	}

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
		if (!rgbodies.size())
			return;

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
        constraint->SetConstrainRotation(true);
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

    if (pickTarget == nullptr)
        return;
    if (pickPullNode == nullptr)
        return;


    Node* pickSource = pickPullNode->GetChild("PickPullSurfaceNode");

    if (pickSource == nullptr)
        return;

    pickPullNode->GetComponent<NewtonKinematicsControllerConstraint>()->SetOtherPosition(pickTarget->GetWorldPosition());
    pickPullNode->GetComponent<NewtonKinematicsControllerConstraint>()->SetOtherRotation(cameraNode_->GetWorldRotation() );

}








void PhysicsTests::InitTouchInput()
{
	touchEnabled_ = true;

	ResourceCache* cache = GetSubsystem<ResourceCache>();
	Input* input = GetSubsystem<Input>();
	XMLFile* layout = cache->GetResource<XMLFile>("UI/ScreenJoystick_Samples.xml");
	const ea::string& patchString = GetScreenJoystickPatchString();
	if (!patchString.empty())
	{
		// Patch the screen joystick layout further on demand
		SharedPtr<XMLFile> patchFile(new XMLFile(context_));
		if (patchFile->FromString(patchString))
			layout->Patch(patchFile);
	}
	screenJoystickIndex_ = (unsigned)input->AddScreenJoystick(layout, cache->GetResource<XMLFile>("UI/DefaultStyle.xml"));
	input->SetScreenJoystickVisible(screenJoystickSettingsIndex_, true);
}

void PhysicsTests::InitMouseMode(MouseMode mode)
{
	useMouseMode_ = mode;

	Input* input = GetSubsystem<Input>();

	if (GetPlatform() != "Web")
	{
		if (useMouseMode_ == MM_FREE)
			input->SetMouseVisible(true);

		if (useMouseMode_ != MM_ABSOLUTE)
		{
			input->SetMouseMode(useMouseMode_);
#if URHO3D_SYSTEMUI
			Console* console = GetSubsystem<Console>();
			if (console && console->IsVisible())
				input->SetMouseMode(MM_ABSOLUTE, true);
#endif
		}
	}
	else
	{
		input->SetMouseVisible(true);
		SubscribeToEvent(E_MOUSEBUTTONDOWN, URHO3D_HANDLER(PhysicsTests, HandleMouseModeRequest));
		SubscribeToEvent(E_MOUSEMODECHANGED, URHO3D_HANDLER(PhysicsTests, HandleMouseModeChange));
	}
}

void PhysicsTests::SetLogoVisible(bool enable)
{
	if (logoSprite_)
		logoSprite_->SetVisible(enable);
}


void PhysicsTests::SetWindowTitleAndIcon()
{
	ResourceCache* cache = GetSubsystem<ResourceCache>();
	Graphics* graphics = GetSubsystem<Graphics>();
	graphics->SetWindowTitle("PhysicsTests");
}

void PhysicsTests::CreateConsoleAndDebugHud()
{
	// Create console
	Console* console = context_->GetSubsystem<Engine>()->CreateConsole();

	// Create debug HUD.
	DebugHud* debugHud = context_->GetSubsystem<Engine>()->CreateDebugHud();
}


void PhysicsTests::HandleKeyUp(StringHash /*eventType*/, VariantMap& eventData)
{
}

void PhysicsTests::HandleKeyDown(StringHash /*eventType*/, VariantMap& eventData)
{
	using namespace KeyDown;

	int key = eventData[P_KEY].GetInt();

	// Toggle console with F1 or backquote
#if URHO3D_SYSTEMUI
	if (key == KEY_F1 || key == KEY_BACKQUOTE)
	{
#if URHO3D_RMLUI
		if (auto* ui = GetSubsystem<RmlUI>())
		{
			if (ui->IsInputCaptured())
				return;
		}
#endif
		if (auto* ui = GetSubsystem<UI>())
		{
			if (UIElement* element = ui->GetFocusElement())
			{
				if (element->IsEditable())
					return;
			}
		}
		GetSubsystem<Console>()->Toggle();
		return;
	}
	// Toggle debug HUD with F2
	else if (key == KEY_F2)
	{
		context_->GetSubsystem<Engine>()->CreateDebugHud()->ToggleAll();
		return;
	}
#endif

	// Common rendering quality controls, only when UI has no focused element
	if (!GetSubsystem<UI>()->GetFocusElement())
	{
		Renderer* renderer = GetSubsystem<Renderer>();

		// Preferences / Pause
		if (key == KEY_SELECT && touchEnabled_)
		{
			paused_ = !paused_;

			Input* input = GetSubsystem<Input>();
			if (screenJoystickSettingsIndex_ == M_MAX_UNSIGNED)
			{
				// Lazy initialization
				ResourceCache* cache = GetSubsystem<ResourceCache>();
				screenJoystickSettingsIndex_ = (unsigned)input->AddScreenJoystick(cache->GetResource<XMLFile>("UI/ScreenJoystickSettings_Samples.xml"), cache->GetResource<XMLFile>("UI/DefaultStyle.xml"));
			}
			else
				input->SetScreenJoystickVisible(screenJoystickSettingsIndex_, paused_);
		}

		// Texture quality
		else if (key == '1')
		{
			auto quality = (unsigned)renderer->GetTextureQuality();
			++quality;
			if (quality > QUALITY_HIGH)
				quality = QUALITY_LOW;
			renderer->SetTextureQuality((MaterialQuality)quality);
		}

		// Material quality
		else if (key == '2')
		{
			auto quality = (unsigned)renderer->GetMaterialQuality();
			++quality;
			if (quality > QUALITY_HIGH)
				quality = QUALITY_LOW;
			renderer->SetMaterialQuality((MaterialQuality)quality);
		}

		// Specular lighting
		else if (key == '3')
			renderer->SetSpecularLighting(!renderer->GetSpecularLighting());

		// Shadow rendering
		else if (key == '4')
			renderer->SetDrawShadows(!renderer->GetDrawShadows());

		// Shadow map resolution
		else if (key == '5')
		{
			int shadowMapSize = renderer->GetShadowMapSize();
			shadowMapSize *= 2;
			if (shadowMapSize > 2048)
				shadowMapSize = 512;
			renderer->SetShadowMapSize(shadowMapSize);
		}

		// Shadow depth and filtering quality
		else if (key == '6')
		{
			ShadowQuality quality = renderer->GetShadowQuality();
			quality = (ShadowQuality)(quality + 1);
			if (quality > SHADOWQUALITY_BLUR_VSM)
				quality = SHADOWQUALITY_SIMPLE_16BIT;
			renderer->SetShadowQuality(quality);
		}

		// Occlusion culling
		else if (key == '7')
		{
			bool occlusion = renderer->GetMaxOccluderTriangles() > 0;
			occlusion = !occlusion;
			renderer->SetMaxOccluderTriangles(occlusion ? 5000 : 0);
		}

		// Instancing
		else if (key == '8')
			renderer->SetDynamicInstancing(!renderer->GetDynamicInstancing());

		// Take screenshot
		else if (key == '9')
		{
			Graphics* graphics = GetSubsystem<Graphics>();
			Image screenshot(context_);
			graphics->TakeScreenShot(screenshot);
			// Here we save in the Data folder with date and time appended
			screenshot.SavePNG(GetSubsystem<FileSystem>()->GetProgramDir() + "Data/Screenshot_" +
				Time::GetTimeStamp().replaced(':', '_').replaced('.', '_').replaced(' ', '_') + ".png");
		}
	}
}

void PhysicsTests::HandleSceneUpdate(StringHash /*eventType*/, VariantMap& eventData)
{
	// Move the camera by touch, if the camera node is initialized by descendant sample class
	if (touchEnabled_ && cameraNode_)
	{
		Input* input = GetSubsystem<Input>();
		for (unsigned i = 0; i < input->GetNumTouches(); ++i)
		{
			TouchState* state = input->GetTouch(i);
			if (!state->touchedElement_)    // Touch on empty space
			{
				if (state->delta_.x_ || state->delta_.y_)
				{
					Camera* camera = cameraNode_->GetComponent<Camera>();
					if (!camera)
						return;

					Graphics* graphics = GetSubsystem<Graphics>();
					yaw_ += TOUCH_SENSITIVITY * camera->GetFov() / graphics->GetHeight() * state->delta_.x_;
					pitch_ += TOUCH_SENSITIVITY * camera->GetFov() / graphics->GetHeight() * state->delta_.y_;

					// Construct new orientation for the camera scene node from yaw and pitch; roll is fixed to zero
					cameraNode_->SetRotation(Quaternion(pitch_, yaw_, 0.0f));
				}
				else
				{
					// Move the cursor to the touch position
					Cursor* cursor = GetSubsystem<UI>()->GetCursor();
					if (cursor && cursor->IsVisible())
						cursor->SetPosition(state->position_);
				}
			}
		}
	}
}

void PhysicsTests::HandleTouchBegin(StringHash /*eventType*/, VariantMap& eventData)
{
	// On some platforms like Windows the presence of touch input can only be detected dynamically
	InitTouchInput();
	UnsubscribeFromEvent("TouchBegin");
}

// If the user clicks the canvas, attempt to switch to relative mouse mode on web platform
void PhysicsTests::HandleMouseModeRequest(StringHash /*eventType*/, VariantMap& eventData)
{
#if URHO3D_SYSTEMUI
	Console* console = GetSubsystem<Console>();
	if (console && console->IsVisible())
		return;
#endif
	Input* input = GetSubsystem<Input>();
	if (useMouseMode_ == MM_ABSOLUTE)
		input->SetMouseVisible(false);
	else if (useMouseMode_ == MM_FREE)
		input->SetMouseVisible(true);
	input->SetMouseMode(useMouseMode_);
}

void PhysicsTests::HandleMouseModeChange(StringHash /*eventType*/, VariantMap& eventData)
{
	Input* input = GetSubsystem<Input>();
	bool mouseLocked = eventData[MouseModeChanged::P_MOUSELOCKED].GetBool();
	input->SetMouseVisible(!mouseLocked);
}

void PhysicsTests::CloseSample()
{
	VariantMap args;
	{
		using namespace KeyDown;
		args[P_KEY] = KEY_ESCAPE;
		args[P_SCANCODE] = SCANCODE_ESCAPE;
		args[P_BUTTONS] = 0;
		args[P_QUALIFIERS] = 0;
		args[P_REPEAT] = false;
		SendEvent(E_KEYDOWN, args);
	}
	{
		using namespace KeyUp;
		args[P_KEY] = KEY_ESCAPE;
		args[P_SCANCODE] = SCANCODE_ESCAPE;
		args[P_BUTTONS] = 0;
		args[P_QUALIFIERS] = 0;
		SendEvent(E_KEYUP, args);
	}
}
