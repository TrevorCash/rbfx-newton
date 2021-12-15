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



		float wheelFriction = 2.0f;

		//backwheel
		Vector3 backWheelOffset = Vector3(-2.0, -0.5, 0);
		Node* backWheel = SpawnSamplePhysicsChamferCylinder(rootNode, Vector3::ZERO + backWheelOffset, 0.8f, 0.2f);
		backWheel->SetWorldRotation(Quaternion(90, 0, 0));
		backWheel->GetComponent<NewtonRigidBody>()->SetCollisionOverride(C->GetComponent<NewtonRigidBody>(), false);
		backWheel->GetComponent<NewtonRigidBody>()->SetUseGyroscopicTorque(enableGyroOnWheels);
		backWheel->GetDerivedComponent<NewtonCollisionShape>()->SetFriction(wheelFriction);


		NewtonHingeConstraint* motor = backWheel->CreateComponent<NewtonHingeConstraint>();
		motor->SetPowerMode(NewtonHingeConstraint::MOTOR_SPEED);
		motor->SetOtherBody(C->GetComponent<NewtonRigidBody>());
		motor->SetWorldPosition(Vector3::ZERO + backWheelOffset);
		motor->SetWorldRotation(Quaternion(0, 90, 0));
		motor->SetMotorTargetAngularRate(10);
		motor->SetMaxTorque(motor->GetMaxTorque()*0.00125f);


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

		motors.push_back(motor);
		motors.push_back(hinge);

		rootNode->SetWorldPosition(worldPos);


	}


	virtual void ResizeVectors()
	{
		stateVec.resize(2);
		actionVec.resize(2);
	}

	virtual void FormResponses()
	{
		GYM::FormResponses();
		GymClient* GymCli = context_->GetSubsystem<GymClient>();

		stateVec[0] = motors[0]->GetCurrentAngularRate();
		stateVec[1] = motors[1]->GetCurrentAngle();

		reward = bodyNode->GetComponent<NewtonRigidBody>()->GetLinearVelocity().x_;

	}

	virtual void ApplyActionVec()
	{
		GYM::ApplyActionVec();
		for (int v = 0; v < actionVec.size(); v++)
		{
			motors[v]->SetMotorTorque(actionVec[v] * 10);
		}
	}


	ea::vector<NewtonHingeConstraint*> motors;
	WeakPtr<Node> bodyNode;
};