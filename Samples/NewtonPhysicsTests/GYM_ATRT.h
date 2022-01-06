#pragma once

#include "GYM.h"



using namespace Urho3D;

class GYM_ATRT : public GYM
{
public:
	URHO3D_OBJECT(GYM_ATRT, GYM);

	explicit GYM_ATRT(Context* context) : GYM(context)
	{

	}



	virtual void Reset()
	{
		GYM::Reset();

		targetWorldVel = Vector3(-1,0,0);
		rootNode = scene_->CreateChild("ATRT");
		rootNode->AddTag("ATRT");
		motors.clear();
		//Body
		bodyNode = SpawnSamplePhysicsBox(rootNode, Vector3::ZERO, Vector3(1, 1, 1));


		//LEFT LEG
		Node* HIP_LEFT = SpawnSamplePhysicsCylinder(rootNode, Vector3(0.0, -0.5, -0.5), 0.5, 0.25);
		HIP_LEFT->Rotate(Quaternion(90, Vector3(1, 0, 0)));

		NewtonHingeConstraint* HIPBODYJOINT_LEFT = bodyNode->CreateComponent<NewtonHingeConstraint>();
		HIPBODYJOINT_LEFT->SetRotation(Quaternion(90, Vector3(0, 1, 0)));
		HIPBODYJOINT_LEFT->SetPosition(Vector3(0.0, -0.5, -0.5));
		HIPBODYJOINT_LEFT->SetOtherBody(HIP_LEFT->GetComponent<NewtonRigidBody>());

		NewtonHingeConstraint* HIPBODYJOINT_LEFT_R = bodyNode->CreateComponent<NewtonHingeConstraint>();
		HIPBODYJOINT_LEFT_R->SetRotation(Quaternion(90, Vector3(0, 1, 0)));
		HIPBODYJOINT_LEFT_R->SetPosition(Vector3(0.0, -0.5, -0.5));
		HIPBODYJOINT_LEFT_R->SetOtherBody(HIP_LEFT->GetComponent<NewtonRigidBody>());



		Node* KNEE_LEFT = SpawnSamplePhysicsCylinder(HIP_LEFT, Vector3(0.5, -1.5, -0.5), 0.3, 0.25);
		KNEE_LEFT->RemoveComponent<NewtonRigidBody>();

		Node* KNEE2_LEFT = SpawnSamplePhysicsCylinder(rootNode, Vector3(0.5, -1.5, -0.75), 0.3, 0.25);
		KNEE2_LEFT->Rotate(Quaternion(90, Vector3(1, 0, 0)));

		NewtonHingeConstraint* KNEEJOINT_LEFT = HIP_LEFT->CreateComponent<NewtonHingeConstraint>();
		KNEEJOINT_LEFT->SetRotation(Quaternion(90, Vector3(0, 0, 1)));
		KNEEJOINT_LEFT->SetWorldPosition(KNEE_LEFT->GetWorldPosition());
		KNEEJOINT_LEFT->SetOtherBody(KNEE2_LEFT->GetComponent<NewtonRigidBody>());

		NewtonHingeConstraint* KNEEJOINT_LEFT_R = HIP_LEFT->CreateComponent<NewtonHingeConstraint>();
		KNEEJOINT_LEFT_R->SetRotation(Quaternion(90, Vector3(0, 0, 1)));
		KNEEJOINT_LEFT_R->SetWorldPosition(KNEE_LEFT->GetWorldPosition());
		KNEEJOINT_LEFT_R->SetOtherBody(KNEE2_LEFT->GetComponent<NewtonRigidBody>());







		Node* KNEE3_LEFT = SpawnSamplePhysicsCylinder(KNEE2_LEFT, Vector3(0.5, -2.5, -0.5), 0.3, 0.25);
		KNEE3_LEFT->RemoveComponent<NewtonRigidBody>();

		Node* KNEE4_LEFT = SpawnSamplePhysicsCylinder(rootNode, Vector3(0.5, -2.5, -0.75), 0.3, 0.25);
		KNEE4_LEFT->Rotate(Quaternion(90, Vector3(1, 0, 0)));

		NewtonHingeConstraint* KNEEJOINT2_LEFT = KNEE2_LEFT->CreateComponent<NewtonHingeConstraint>();
		KNEEJOINT2_LEFT->SetRotation(Quaternion(90, Vector3(0, 0, 1)));
		KNEEJOINT2_LEFT->SetWorldPosition(KNEE3_LEFT->GetWorldPosition());
		KNEEJOINT2_LEFT->SetOtherBody(KNEE4_LEFT->GetComponent<NewtonRigidBody>());

		NewtonHingeConstraint* KNEEJOINT2_LEFT_R = KNEE2_LEFT->CreateComponent<NewtonHingeConstraint>();
		KNEEJOINT2_LEFT_R->SetRotation(Quaternion(90, Vector3(0, 0, 1)));
		KNEEJOINT2_LEFT_R->SetWorldPosition(KNEE3_LEFT->GetWorldPosition());
		KNEEJOINT2_LEFT_R->SetOtherBody(KNEE4_LEFT->GetComponent<NewtonRigidBody>());


		FOOT_LEFT = SpawnSamplePhysicsCylinder(KNEE4_LEFT, Vector3(0, -3, -0.5), 0.2, 0.25);
		FOOT_LEFT->RemoveComponent<NewtonRigidBody>();
		FOOT_LEFT->GetDerivedComponent<NewtonCollisionShape>()->SetFriction(10.0f);

		//RIGHT LEG
		Node* HIP_RIGHT = SpawnSamplePhysicsCylinder(rootNode, Vector3(0.0, -0.5, 0.5), 0.5, 0.25);
		HIP_RIGHT->Rotate(Quaternion(90, Vector3(1, 0, 0)));

		NewtonHingeConstraint* HIPBODYJOINT_RIGHT = bodyNode->CreateComponent<NewtonHingeConstraint>();
		HIPBODYJOINT_RIGHT->SetRotation(Quaternion(90, Vector3(0, 1, 0)));
		HIPBODYJOINT_RIGHT->SetPosition(Vector3(0.0, -0.5, 0.5));
		HIPBODYJOINT_RIGHT->SetOtherBody(HIP_RIGHT->GetComponent<NewtonRigidBody>());

		NewtonHingeConstraint* HIPBODYJOINT_RIGHT_R = bodyNode->CreateComponent<NewtonHingeConstraint>();
		HIPBODYJOINT_RIGHT_R->SetRotation(Quaternion(90, Vector3(0, 1, 0)));
		HIPBODYJOINT_RIGHT_R->SetPosition(Vector3(0.0, -0.5, 0.5));
		HIPBODYJOINT_RIGHT_R->SetOtherBody(HIP_RIGHT->GetComponent<NewtonRigidBody>());


		Node* KNEE_RIGHT = SpawnSamplePhysicsCylinder(HIP_RIGHT, Vector3(0.5, -1.5, 0.75), 0.3, 0.25);
		KNEE_RIGHT->RemoveComponent<NewtonRigidBody>();

		Node* KNEE2_RIGHT = SpawnSamplePhysicsCylinder(rootNode, Vector3(0.5, -1.5, 0.5), 0.3, 0.25);
		KNEE2_RIGHT->Rotate(Quaternion(90, Vector3(1, 0, 0)));

		NewtonHingeConstraint* KNEEJOINT_RIGHT = HIP_RIGHT->CreateComponent<NewtonHingeConstraint>();
		KNEEJOINT_RIGHT->SetRotation(Quaternion(90, Vector3(0, 0, 1)));
		KNEEJOINT_RIGHT->SetWorldPosition(KNEE_RIGHT->GetWorldPosition());
		KNEEJOINT_RIGHT->SetOtherBody(KNEE2_RIGHT->GetComponent<NewtonRigidBody>());

		NewtonHingeConstraint* KNEEJOINT_RIGHT_R = HIP_RIGHT->CreateComponent<NewtonHingeConstraint>();
		KNEEJOINT_RIGHT_R->SetRotation(Quaternion(90, Vector3(0, 0, 1)));
		KNEEJOINT_RIGHT_R->SetWorldPosition(KNEE_RIGHT->GetWorldPosition());
		KNEEJOINT_RIGHT_R->SetOtherBody(KNEE2_RIGHT->GetComponent<NewtonRigidBody>());


		Node* KNEE3_RIGHT = SpawnSamplePhysicsCylinder(KNEE2_RIGHT, Vector3(0.5, -2.5, 0.75), 0.3, 0.25);
		KNEE3_RIGHT->RemoveComponent<NewtonRigidBody>();

		Node* KNEE4_RIGHT = SpawnSamplePhysicsCylinder(rootNode, Vector3(0.5, -2.5, 0.5), 0.3, 0.25);
		KNEE4_RIGHT->Rotate(Quaternion(90, Vector3(1, 0, 0)));

		NewtonHingeConstraint* KNEEJOINT2_RIGHT = KNEE2_RIGHT->CreateComponent<NewtonHingeConstraint>();
		KNEEJOINT2_RIGHT->SetRotation(Quaternion(90, Vector3(0, 0, 1)));
		KNEEJOINT2_RIGHT->SetWorldPosition(KNEE3_RIGHT->GetWorldPosition());
		KNEEJOINT2_RIGHT->SetOtherBody(KNEE4_RIGHT->GetComponent<NewtonRigidBody>());

		NewtonHingeConstraint* KNEEJOINT2_RIGHT_R = KNEE2_RIGHT->CreateComponent<NewtonHingeConstraint>();
		KNEEJOINT2_RIGHT_R->SetRotation(Quaternion(90, Vector3(0, 0, 1)));
		KNEEJOINT2_RIGHT_R->SetWorldPosition(KNEE3_RIGHT->GetWorldPosition());
		KNEEJOINT2_RIGHT_R->SetOtherBody(KNEE4_RIGHT->GetComponent<NewtonRigidBody>());

		FOOT_RIGHT = SpawnSamplePhysicsCylinder(KNEE4_RIGHT, Vector3(0, -3, 0.5), 0.2, 0.25);
		FOOT_RIGHT->RemoveComponent<NewtonRigidBody>();
		FOOT_RIGHT->GetDerivedComponent<NewtonCollisionShape>()->SetFriction(10.0f);


		NewtonHingeConstraint::PoweredMode powerMode = NewtonHingeConstraint::MOTOR_TORQUE;
		HIPBODYJOINT_LEFT->SetPowerMode(powerMode);
		HIPBODYJOINT_RIGHT->SetPowerMode(powerMode);
		KNEEJOINT_LEFT->SetPowerMode(powerMode);
		KNEEJOINT_RIGHT->SetPowerMode(powerMode);
		KNEEJOINT2_LEFT->SetPowerMode(powerMode);
		KNEEJOINT2_RIGHT->SetPowerMode(powerMode);

		motors.push_back(HIPBODYJOINT_LEFT);
		motors.push_back(HIPBODYJOINT_RIGHT);
		motors.push_back(KNEEJOINT_LEFT);
		motors.push_back(KNEEJOINT_RIGHT);
		motors.push_back(KNEEJOINT2_LEFT);
		motors.push_back(KNEEJOINT2_RIGHT);

		rootNode->SetWorldPosition(worldPos);
		//rootNode->Rotate(Quaternion(RandomNormal(0, 45), Vector3(0, 0, 1)));
	}


	virtual void ResizeVectors()
	{
		actionVec.resize(6);
		actionVec_1 = actionVec;

		FormResponses(0.0f);
		stateVec_1 = stateVec;
	}

	virtual void FormResponses(float timeStep)
	{
		GYM::FormResponses(timeStep);
		GymClient* GymCli = context_->GetSubsystem<GymClient>();


		int vertState = SetNextState((bodyNode->GetWorldPosition().y_ - 2.0f)*0.5f);//Vertical Translation

		//linear velocity
		//Vector3 worldVel = bodyNode->GetComponent<NewtonRigidBody>()->GetLinearVelocity();
		//SetNextState(worldVel.x_ / 10.0f);                     
		//SetNextState(worldVel.y_ / 10.0f);
		//SetNextState(worldVel.z_ / 10.0f);

		//local linear velocity
		Vector3 vel = bodyNode->GetComponent<NewtonRigidBody>()->GetLinearVelocity(Urho3D::TS_LOCAL);
		SetNextState(vel.x_ / 10.0f);
		SetNextState(vel.y_ / 10.0f);
		SetNextState(vel.z_ / 10.0f);



		int fbtilt = SetNextState(bodyNode->GetWorldRight().y_);    //forward-back tilt indicator
		int rolltilt = SetNextState(bodyNode->GetWorldDirection().y_);//roll indicator


		SetNextState(FOOT_LEFT->GetWorldPosition().y_);
		SetNextState(FOOT_RIGHT->GetWorldPosition().y_);
		
		SetNextRewardPart(Sign(stateVec[vertState])*100.0*stateVec[vertState]*stateVec[vertState]);//vertical displacement
		//SetNextRewardPart(-0.1*Abs(stateVec[fbtilt]));
		//SetNextRewardPart(-0.1*Abs(stateVec[rolltilt]));


		//SetNextState(targetWorldVel.x_);
		//SetNextState(targetWorldVel.z_);

		for (int i = 0; i < motors.size(); i++)
		{
			//SetNextState(motors[i]->GetOwnBody()->GetNode()->GetPosition().x_);
			//SetNextState(motors[i]->GetOwnBody()->GetNode()->GetPosition().y_);

			SetNextState(sin(motors[i]->GetCurrentAngle()));
			SetNextState(cos(motors[i]->GetCurrentAngle()));

			SetNextState(motors[i]->GetCurrentAngularRate()/10.0f);
		}


		//BuildStateDerivatives(timeStep);





		//float velocityAgreement = worldVel.DotProduct(targetWorldVel.Normalized()) / targetWorldVel.Length();
		//SetNextRewardPart(10*velocityAgreement);
		SetNextRewardPart(0.1*timeStep);
		SetNextRewardPart(-(Abs(actionVec_1[0])+
			Abs(actionVec_1[1]) +
				Abs(actionVec_1[2]) +
					Abs(actionVec_1[3]) +
						Abs(actionVec_1[4]) +
							Abs(actionVec_1[5])));//prev torques



		if (bodyNode->GetWorldPosition().y_ < 1.0f)
		{
			end = 1;
		}


	}

	virtual void ApplyActionVec(float timeStep)
	{
		GYM::ApplyActionVec(timeStep);

		for(int m = 0; m < motors.size(); m++ )
			motors[m]->SetMotorTorque(actionVec[m] * 10);

	}

	virtual void DrawDebugGeometry(DebugRenderer* debugRenderer)
	{
		debugRenderer->AddLine(bodyNode->GetWorldPosition(), bodyNode->GetWorldPosition() + targetWorldVel * 5.0f, Color::RED);
	}

	ea::vector<NewtonHingeConstraint*> motors;
	WeakPtr<Node> bodyNode;
	WeakPtr<Node> FOOT_RIGHT;
	WeakPtr<Node> FOOT_LEFT;
	Vector3 targetWorldVel;
};




