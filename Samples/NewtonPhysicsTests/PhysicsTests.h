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

#pragma once

#include "Samples/Sample.h"
#include "NewtonConstraint.h"

namespace Urho3D
{
class Node;
class Scene;
class NewtonHingeConstraint;
}
/// PhysicsTests example.
/// This sample demonstrates different types of physics configurations and provides a testing ground for physics functionality.
class PhysicsTests : public Application
{
    URHO3D_OBJECT(PhysicsTests, Application);

public:
    /// Construct.
	explicit PhysicsTests(Context* context);

	void Setup() override
	{
		// Engine is not initialized yet. Set up all the parameters now.
		engineParameters_[EP_FULL_SCREEN] = false;
		engineParameters_[EP_WINDOW_HEIGHT] = 600;
		engineParameters_[EP_WINDOW_WIDTH] = 800;
		// Resource prefix path is a list of semicolon-separated paths which will be checked for containing resource directories. They are relative to application executable file.
		engineParameters_[EP_RESOURCE_PREFIX_PATHS] = ".;..";
	}


    /// Setup after engine initialization and before running the main loop.
	virtual void Start();
	virtual void Start(const ea::vector<ea::string>& args);

protected:
    /// Return XML patch instructions for screen joystick layout for a specific sample app, if any.
    ea::string GetScreenJoystickPatchString() const  { return
        "<patch>"
        "    <remove sel=\"/element/element[./attribute[@name='Name' and @value='Button0']]/attribute[@name='Is Visible']\" />"
        "    <replace sel=\"/element/element[./attribute[@name='Name' and @value='Button0']]/element[./attribute[@name='Name' and @value='Label']]/attribute[@name='Text']/@value\">Spawn</replace>"
        "    <add sel=\"/element/element[./attribute[@name='Name' and @value='Button0']]\">"
        "        <element type=\"Text\">"
        "            <attribute name=\"Name\" value=\"MouseButtonBinding\" />"
        "            <attribute name=\"Text\" value=\"LEFT\" />"
        "        </element>"
        "    </add>"
        "    <remove sel=\"/element/element[./attribute[@name='Name' and @value='Button1']]/attribute[@name='Is Visible']\" />"
        "    <replace sel=\"/element/element[./attribute[@name='Name' and @value='Button1']]/element[./attribute[@name='Name' and @value='Label']]/attribute[@name='Text']/@value\">Debug</replace>"
        "    <add sel=\"/element/element[./attribute[@name='Name' and @value='Button1']]\">"
        "        <element type=\"Text\">"
        "            <attribute name=\"Name\" value=\"KeyBinding\" />"
        "            <attribute name=\"Text\" value=\"SPACE\" />"
        "        </element>"
        "    </add>"
        "</patch>";
    }

private:
    /// Construct the scene content.
    void CreateScene();


    void CreatePyramids(Vector3 position);


    void CreateTowerOfLiar(Vector3 position);
    /// Construct an instruction text to the UI.
    void CreateInstructions();
    /// Set up a viewport for displaying the scene.
    void SetupViewport();
    /// Subscribe to application-wide logic update and post-render update events.
    void SubscribeToEvents();





    /// Read input and moves the camera.
    void MoveCamera(float timeStep);





    /// Spawn a physics object from the camera position.
    void SpawnObject();
    void SpawnConvexHull(const Vector3& worldPos);
    void SpawnCompound(const Vector3& worldPos);
    void SpawnDecompCompound(const Vector3& worldPos);    
	void SpawnSceneCompoundTest(const Vector3& worldPos, bool oneBody);
    void SpawnNSquaredJointedObject(Vector3 worldPosition);
    void SpawnGlueJointedObject(Vector3 worldPosition);
    void SpawnLinearJointedObject(float size, Vector3 worldPosition);
    void SpawnMaterialsTest(Vector3 worldPosition);
    void SpawnBallSocketTest(Vector3 worldPosition);
    void SpawnHingeActuatorTest(Vector3 worldPosition);
    void SpawnCollisionExceptionsTest(Vector3 worldPosition);
    void SpawnSliderTest(Vector3 worldPosition);
	void SpawnGearTest(Vector3 worldPosition);
    void SpawnRandomObjects();
    void SpawnCompoundedRectTest(Vector3 worldPosition);
    void SpawnCompoundedRectTest2(Vector3 worldPosition);
    void SpawnTrialBike(Vector3 worldPosition, Quaternion orientation, bool enableGyroOnWheels);
    void SpawnHingeSpringTest(const Vector3 ZERO, const Quaternion IDENTITY);
    void SpawnKinematicBodyTest(Vector3 worldPosition, Quaternion worldRotation);
	void SpawnRejointingTest(Vector3 worldPosition);
	void SpawnCollisionOffsetTest(Vector3 worldPosition);
	void SpawnATRT(Vector3 worldPosition);
    void SpawnSegway(Vector3 worldPosition);


    /// Handle the logic update event.
    void HandleUpdate(StringHash eventType, VariantMap& eventData);
    /// Handle the post-render update event.
    void HandlePostRenderUpdate(StringHash eventType, VariantMap& eventData);
    /// Handle physics pre-step
    void HandlePhysicsPreStep(StringHash eventType, VariantMap& eventData);

    /// Handle physics post-step
    void HandlePhysicsPostStep(StringHash eventType, VariantMap& eventData);

    /// Flag for drawing debug geometry.
    bool drawDebug_ = false;
    void DecomposePhysicsTree();
    void RecomposePhysicsTree();
    void  TransportNode();



    void HandleMouseButtonUp(StringHash eventType, VariantMap& eventData);
    void HandleMouseButtonDown(StringHash eventType, VariantMap& eventData);


    void HandleCollisionStart(StringHash eventType, VariantMap& eventData);
    void HandleCollision(StringHash eventType, VariantMap& eventData);
    void HandleCollisionEnd(StringHash eventType, VariantMap& eventData);


    Node* pickPullNode = nullptr;
    Quaternion pickPullCameraStartOrientation;
    Vector3 pickPullStartPositionWorld;
    Vector3 pickPullEndPositionWorld;
    NewtonHingeConstraint* hingeActuatorTest = nullptr;
    float timeAccum = 0.0f;



	Node* reJointRoot = nullptr;
	Node* reJointA = nullptr;
	Node* reJointB = nullptr;
	void ToggleRejointTest();



    void CreatePickTargetNodeOnPhysics();
    void ReleasePickTargetOnPhysics();
    void UpdatePickPull();
    RayQueryResult GetCameraPickNode();

    //temp variable to tracking world position across updates.
    ea::vector<float> worldPosHistory_;
	ea::vector<float> worldPosHistory2_;


	ea::vector<NewtonConstraint*> forceCalculationConstraints_;

    WeakPtr<Node> kinematicNode_;

    void CreateScenery(Vector3 worldPosition);
    void RemovePickNode(bool removeRigidBodyOnly = false);
   
   


	protected:

		/// Initialize touch input on mobile platform.
		void InitTouchInput();
		/// Initialize mouse mode on non-web platform.
		void InitMouseMode(MouseMode mode);
		/// Control logo visibility.
		void SetLogoVisible(bool enable);
		///
		void CloseSample();

		/// Logo sprite.
		SharedPtr<Sprite> logoSprite_;
		/// Scene.
		SharedPtr<Scene> scene_;
		/// Camera scene node.
		SharedPtr<Node> cameraNode_;
		/// Camera yaw angle.
		float yaw_;
		/// Camera pitch angle.
		float pitch_;
		/// Flag to indicate whether touch input has been enabled.
		bool touchEnabled_;
		/// Mouse mode option to use in the sample.
		MouseMode useMouseMode_;

private:

	/// Set custom window Title & Icon
	void SetWindowTitleAndIcon();
	/// Create console and debug HUD.
	void CreateConsoleAndDebugHud();
	/// Handle request for mouse mode on web platform.
	void HandleMouseModeRequest(StringHash eventType, VariantMap& eventData);
	/// Handle request for mouse mode change on web platform.
	void HandleMouseModeChange(StringHash eventType, VariantMap& eventData);
	/// Handle key down event to process key controls common to all samples.
	void HandleKeyDown(StringHash eventType, VariantMap& eventData);
	/// Handle key up event to process key controls common to all samples.
	void HandleKeyUp(StringHash eventType, VariantMap& eventData);
	/// Handle scene update event to control camera's pitch and yaw for all samples.
	void HandleSceneUpdate(StringHash eventType, VariantMap& eventData);
	/// Handle touch begin event to initialize touch input on desktop platform.
	void HandleTouchBegin(StringHash eventType, VariantMap& eventData);

	/// Screen joystick index for navigational controls (mobile platforms only).
	unsigned screenJoystickIndex_;
	/// Screen joystick index for settings (mobile platforms only).
	unsigned screenJoystickSettingsIndex_;
	/// Pause flag.
	bool paused_;













};

// A helper macro which defines main function. Forgetting it will result in linker errors complaining about missing `_main` or `_WinMain@16`.
URHO3D_DEFINE_APPLICATION_MAIN(PhysicsTests);