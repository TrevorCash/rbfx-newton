#pragma once

#include "GYM.h"



using namespace Urho3D;

class GYM_UniCycle : public GYM
{
public:
	URHO3D_OBJECT(GYM_UniCycle, GYM);

	explicit GYM_UniCycle(Context* context) : GYM(context)
	{

	}



	virtual void Reset()
	{
		GYM::Reset();
		timeLimit = 30.0f;


		rootNode = scene_->CreateChild("UniCycle");
		rootNode->AddTag("UniCycle");
		motors.clear();

		rootNode = scene_->CreateChild("UniCycle");

		bodyNode = SpawnSamplePhysicsBox(rootNode, Vector3::ZERO, Vector3(0.5, 4, 0.5));

		Node* Wheel = SpawnSamplePhysicsCylinder(rootNode, Vector3(0, -3, 0), 1.0, 0.5);
		Wheel->Rotate(Quaternion(90, Vector3(1, 0, 0)));
		Wheel->GetDerivedComponent<NewtonCollisionShape>()->SetFriction(20);

		NewtonHingeConstraint* motor = bodyNode->CreateComponent<NewtonHingeConstraint>();
		motor->SetRotation(Quaternion(90, Vector3(0, 1, 0)));
		motor->SetPosition(Vector3(0, -3, 0));
		motor->SetEnableLimits(false);
		motor->SetPowerMode(NewtonHingeConstraint::MOTOR_TORQUE);
		motor->SetOtherBody(Wheel->GetComponent<NewtonRigidBody>());
		motor->SetMotorTorque(1);


		motors.push_back(motor);

		rootNode->SetWorldPosition(worldPos);
		rootNode->Rotate(Quaternion(Random(-30, 30), Vector3(0, 0, 1)));

		targetX = Random(-10, 10);
	}


	virtual void ResizeVectors()
	{
		actionVec.resize(1);
		stateVec.resize(10);

		actionVec_1 = actionVec;
		stateVec_1 = stateVec;


		rewardParts.resize(2);
	}

	virtual void FormResponses(float timeStep)
	{
		GYM::FormResponses(timeStep);
		GymClient* GymCli = context_->GetSubsystem<GymClient>();

		float errorDist = (bodyNode->GetWorldPosition().x_ - targetX)/10.0f;

		stateVec[0] = (bodyNode->GetWorldPosition().y_ - 5.0f)/5.0f;//Vertical Translation
		stateVec[1] = motors[0]->GetCurrentAngularRate() / 10.0f;
		stateVec[2] = bodyNode->GetWorldRight().y_;//angle indicator
		stateVec[3] = bodyNode->GetComponent<NewtonRigidBody>()->GetLinearVelocity().x_/3.0f;
		stateVec[4] = errorDist;


		//prev states for derivative
		stateVec[5] = stateVec_1[0];
		stateVec[6] = stateVec_1[1];
		stateVec[7] = stateVec_1[2];
		stateVec[8] = stateVec_1[3];
		stateVec[9] = stateVec_1[4];


		rewardParts[0] = 10*bodyNode->GetWorldUp().y_*bodyNode->GetWorldUp().y_;//vertical displacement 
		rewardParts[1] = 100-100*Abs(errorDist*errorDist);//keep local to spawn

		if (stateVec[0] < -0.7)
		{
			end = 1;
			rewardParts[0] = -1000.0f;//penalize falling down.
			rewardParts[1] = 0.0f;
		}
	}

	virtual void ApplyActionVec(float timeStep)
	{
		GYM::ApplyActionVec(timeStep);


		motors[0]->SetMotorTorque(actionVec[0] * 20);
	}


	virtual void DrawDebugGeometry(DebugRenderer* debugRenderer)
	{
		GYM::DrawDebugGeometry(debugRenderer);
	
		debugRenderer->AddCross(Vector3(targetX, worldPos.y_, worldPos.z_), 1, Color::GREEN);
	}

	ea::vector<NewtonHingeConstraint*> motors;
	WeakPtr<Node> bodyNode;
	WeakPtr<Node> top;
	float targetX = 0.0f;
};