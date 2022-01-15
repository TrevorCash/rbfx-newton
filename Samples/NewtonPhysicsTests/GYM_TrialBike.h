#pragma once
#include "GYM.h"



using namespace Urho3D;

class GYM_TrialBike : public GYM
{
public:
	URHO3D_OBJECT(GYM_TrialBike, GYM);

	explicit GYM_TrialBike(Context* context) : GYM(context)
	{

	}



	virtual void Reset()
	{
		GYM::Reset();

		timeLimit = 15.0f;
		rootNode = scene_->CreateChild("TrialBike");
		rootNode->AddTag("TrialBike");


		bool enableGyroOnWheels = true;

		//A (Engine Body)
		bodyNode = SpawnSamplePhysicsBox(rootNode, Vector3::ZERO, Vector3(1, 1, 0.5f));

		Node* B = SpawnSamplePhysicsBox(bodyNode, Vector3::ZERO + Vector3(-1, 0.7, 0), Vector3(2, 0.3, 0.5));
		B->RemoveComponent<NewtonRigidBody>();
		B->SetWorldRotation(Quaternion(0, 0, -30));

		Node* C = SpawnSamplePhysicsBox(rootNode, Vector3::ZERO + Vector3(-1, -0.5, 0), Vector3(2, 0.3, 0.5));
		C->SetWorldRotation(Quaternion(0, 0, 0));

		C->GetComponent<NewtonRigidBody>()->SetCollisionOverride(bodyNode->GetComponent<NewtonRigidBody>(), false);

		//make back spring.
		NewtonHingeConstraint* hingeConstraint = bodyNode->CreateComponent<NewtonHingeConstraint>();
		hingeConstraint->SetOtherBody(C->GetComponent<NewtonRigidBody>());
		hingeConstraint->SetNoPowerSpringDamper(true);
		hingeConstraint->SetNoPowerSpringCoefficient(1000.0f);
		hingeConstraint->SetWorldRotation(Quaternion(90, 0, 90));
		hingeConstraint->SetWorldPosition(bodyNode->GetWorldPosition() + Vector3(0, -0.5, 0));



		Node* D = SpawnSamplePhysicsBox(bodyNode, Vector3::ZERO + Vector3(0.7, 0.5, 0), Vector3(1, 0.5, 0.5));
		D->RemoveComponent<NewtonRigidBody>();
		D->SetWorldRotation(Quaternion(0, 0, 45));


		Node* E = SpawnSamplePhysicsBox(rootNode, Vector3::ZERO + Vector3(1.5, 0, 0), Vector3(0.2, 2.5, 0.5));
		E->GetComponent<NewtonRigidBody>()->SetCollisionOverride(bodyNode->GetComponent<NewtonRigidBody>(), false);
		E->SetWorldRotation(Quaternion(0, 0, 20));


		NewtonHingeConstraint* hinge = E->CreateComponent<NewtonHingeConstraint>();
		hinge->SetPowerMode(NewtonHingeConstraint::MOTOR_TORQUE);
		hinge->SetOtherBody(bodyNode->GetComponent<NewtonRigidBody>());
		hinge->SetWorldPosition(Vector3::ZERO + Vector3(1.2, 0.8, 0));
		hinge->SetWorldRotation(Quaternion(0, 0, -90 + 20));

		NewtonHingeConstraint* hingelimits = E->CreateComponent<NewtonHingeConstraint>();
		hingelimits->SetOtherBody(bodyNode->GetComponent<NewtonRigidBody>());
		hingelimits->SetWorldPosition(Vector3::ZERO + Vector3(1.2, 0.8, 0));
		hingelimits->SetWorldRotation(Quaternion(0, 0, -90 + 20));



		Node* F = SpawnSamplePhysicsBox(rootNode, Vector3::ZERO + Vector3(1.5, 0, 0), Vector3(0.2, 2.5, 0.5));
		F->SetWorldRotation(Quaternion(0, 0, 20));
		F->GetComponent<NewtonRigidBody>()->SetCollisionOverride(E->GetComponent<NewtonRigidBody>(), false);
		F->GetComponent<NewtonRigidBody>()->SetCollisionOverride(bodyNode->GetComponent<NewtonRigidBody>(), false);

		NewtonSliderConstraint* frontSuspension = F->CreateComponent<NewtonSliderConstraint>();
		frontSuspension->SetOtherBody(E->GetComponent<NewtonRigidBody>());
		frontSuspension->SetWorldRotation(F->GetWorldRotation() * Quaternion(0, 0, 90));
		frontSuspension->SetEnableSliderSpringDamper(true);
		frontSuspension->SetSliderSpringCoefficient(1000.0f);
		frontSuspension->SetSliderDamperCoefficient(50.0f);
		frontSuspension->SetEnableTwistLimits(true, true);
		frontSuspension->SetTwistLimits(0, 0);
		frontSuspension->SetEnableSliderLimits(true, true);
		frontSuspension->SetSliderLimits(-0.5, 0.5);



		float wheelFriction = 20.0f;

		//backwheel
		Vector3 backWheelOffset = Vector3(-2.0, -0.5, 0);
		Node* backWheel = SpawnSamplePhysicsChamferCylinder(rootNode, Vector3::ZERO + backWheelOffset, 0.8f, 0.2f);
		backWheel->SetWorldRotation(Quaternion(90, 0, 0));
		backWheel->GetComponent<NewtonRigidBody>()->SetCollisionOverride(C->GetComponent<NewtonRigidBody>(), false);
		backWheel->GetComponent<NewtonRigidBody>()->SetUseGyroscopicTorque(enableGyroOnWheels);
		backWheel->GetDerivedComponent<NewtonCollisionShape>()->SetFriction(wheelFriction);


		NewtonHingeConstraint* motor = backWheel->CreateComponent<NewtonHingeConstraint>();
		motor->SetPowerMode(NewtonHingeConstraint::MOTOR_TORQUE);
		motor->SetOtherBody(C->GetComponent<NewtonRigidBody>());
		motor->SetWorldPosition(Vector3::ZERO + backWheelOffset);
		motor->SetWorldRotation(Quaternion(0, 90, 0));
		//motor->SetMotorTargetAngularRate(10);
		motor->SetMotorMaxAngularRate(20);
		//motor->SetMaxTorque(motor->GetMaxTorque()*0.00125f);


		Vector3 frontWheelOffset = Vector3(1.8, -1, 0);
		Node* frontWheel = SpawnSamplePhysicsChamferCylinder(rootNode, Vector3::ZERO + frontWheelOffset, 0.8f, 0.2f);
		frontWheel->SetWorldRotation(Quaternion(90, 0, 0));
		frontWheel->GetComponent<NewtonRigidBody>()->SetCollisionOverride(E->GetComponent<NewtonRigidBody>(), false);
		frontWheel->GetComponent<NewtonRigidBody>()->SetCollisionOverride(F->GetComponent<NewtonRigidBody>(), false);
		frontWheel->GetComponent<NewtonRigidBody>()->SetUseGyroscopicTorque(enableGyroOnWheels);
		frontWheel->GetDerivedComponent<NewtonCollisionShape>()->SetFriction(wheelFriction);


		NewtonHingeConstraint* frontAxle = frontWheel->CreateComponent<NewtonHingeConstraint>();
		//frontAxle->SetPowerMode(NewtonHingeConstraint::MOTOR_TORQUE);
		frontAxle->SetOtherBody(F->GetComponent<NewtonRigidBody>());
		frontAxle->SetWorldPosition(Vector3::ZERO + frontWheelOffset);
		frontAxle->SetWorldRotation(Quaternion(0, 90, 0));
		frontAxle->SetEnableLimits(false);
		//frontAxle->SetMotorTargetAngularRate(10);

		motors.clear();
		
		motors.push_back(hinge);
		motors.push_back(motor);

		rootNode->SetWorldPosition(worldPos - Vector3(0,1.8,0));
		//rootNode->Rotate(Quaternion(Random(-1.0f,1.0f)*20, Vector3(1, 0, 0)));


		targetAngularVel = Vector3(0, Random(-1.0f, 1.0f), 0);
		targetSpeed = Random(1, 1);

	}


	virtual void ResizeVectors()
	{
		actionVec.resize(2);
		actionVec_1 = actionVec;

		FormResponses(0.0f);
		stateVec_1 = stateVec;
	}

	virtual void FormResponses(float timeStep)
	{
		GYM::FormResponses(timeStep);
		
		
		float angularY = bodyNode->GetComponent<NewtonRigidBody>()->GetAngularVelocity().y_;
		float forwardVel = bodyNode->GetComponent<NewtonRigidBody>()->GetLinearVelocity(TS_LOCAL).x_;
		SetNextState(targetAngularVel.y_);
		SetNextState(targetSpeed*0.1f);

		SetNextState(bodyNode->GetWorldDirection().y_);//side to side tilt indicator
		SetNextState(motors[0]->GetCurrentAngle()/(M_PI));
		SetNextState(forwardVel*0.1f);
		SetNextState(angularY);



		BuildStateDerivatives(timeStep);

		//reward for turning with target angular vel
		float turnAgreement = targetAngularVel.y_*angularY;
		SetNextRewardPart(10.0f*turnAgreement);

		//reward for velocity 
		float reward = targetSpeed*forwardVel;
		if (reward < 0.1f)
			reward = -100;

		SetNextRewardPart(10.0f*reward);


		//reward staying vertical
		SetNextRewardPart(10.0f*bodyNode->GetWorldUp().y_);
		
		//reward hint for steering into the lean
		SetNextRewardPart(-10.0f*motors[0]->GetCurrentAngle()*bodyNode->GetWorldDirection().y_);

		//punish hard turn angle
		SetNextRewardPart(-10.0f*Abs(motors[0]->GetCurrentAngle()));

		//energy save backwheel
		SetNextRewardPart(-actionVec_1[0]*actionVec_1[0] - actionVec_1[1]*actionVec_1[1]);

		//guide the bike to lean towards into direction of rotation
		SetNextRewardPart(10*bodyNode->GetWorldDirection().y_*targetAngularVel.y_);

		if (bodyNode->GetWorldUp().y_ <= 0.1f)
		{
			end = 1;
		}
	}

	virtual void ApplyActionVec(float timeStep)
	{
		GYM::ApplyActionVec(timeStep);









		motors[0]->SetMotorTorque(actionVec[0]*3);
		motors[1]->SetMotorTorque(actionVec[1]*10);
	}

	virtual void DrawDebugGeometry(DebugRenderer* debugRenderer)
	{
		debugRenderer->AddLine(bodyNode->GetWorldPosition(), bodyNode->GetWorldPosition() + targetAngularVel*10.0f, Color::RED);
		debugRenderer->AddLine(bodyNode->GetWorldPosition(), bodyNode->GetWorldPosition() + bodyNode->GetComponent<NewtonRigidBody>()->GetAngularVelocity(), Color::BLUE);


		
	}

	ea::vector<NewtonHingeConstraint*> motors;
	WeakPtr<Node> bodyNode;

	Vector3 targetAngularVel;
	float targetSpeed;


};