#pragma once


#include <Urho3D/Core/Object.h>
#include <Urho3D/SystemUI/SystemUI.h>
#include <Urho3D/IO/ArchiveSerialization.h>
#include <Urho3D/Graphics/DebugRenderer.h>
#include <VisualDebugger.h>

using namespace Urho3D;

class GYM : public Object
{
public:
	URHO3D_OBJECT(GYM, Object);

	explicit GYM(Context* context) : Object(context)
	{

	}

	virtual void TearDown()
	{
		if (rootNode.NotNull())
		{
			rootNode->RemoveAllChildren();
			rootNode->Remove();
		}
	}

	virtual void Reset()
	{
		TearDown();
		timeUpCounter = 0;
		end = 0;
		ResizeVectors();
	}

	virtual void PostReset()
	{
		FormResponses(1/60.0);
	}

	virtual void ResizeVectors(){}

	virtual void Update(float timeStep)
	{
		ApplyActionVec( timeStep);
		FormResponses( timeStep);
		FormTotalReward();

		timeUpCounter += timeStep;

		if (timeUpCounter > timeLimit)
			end = 1;
	}

	virtual void FormResponses(float timeStep)
	{
		stateVec_1 = stateVec;
	}

	virtual void ApplyActionVec(float timeStep)
	{
		actionVec_1 = actionVec;
	}

	virtual void FormTotalReward()
	{
		reward = 0;
		for (int i = 0; i < rewardParts.size(); i++)
		{
			reward += rewardParts[i];
		}
	}
	virtual void DrawUIStats()
	{
		ui::Begin("ActionVec");
			
		for (int i = 0; i < actionVec.size(); i++)
		{
			ui::Text("Action[%d]: %f", i, actionVec[i]);
		}

		ui::End();

		ui::Begin("StateVec");

		for (int i = 0; i < stateVec.size(); i++)
		{
			ui::Text("State[%d]: %f", i, stateVec[i]);
		}

		ui::End();

		ui::Begin("Reward");
		for (int i = 0; i < rewardParts.size(); i++)
		{
			ui::Text("Reward[%d]: %f",i, rewardParts[i]);
		}
		ui::Text("Total Reward: %f", reward);
		ui::Text("End: %d", end);
		ui::End();



	}

	virtual void DrawDebugGeometry(DebugRenderer* debugRenderer)
	{

	}

	ea::vector<float> actionVec;
	ea::vector<float> stateVec;

	ea::vector<float> actionVec_1;
	ea::vector<float> stateVec_1;

	ea::vector<float> rewardParts;

	float reward;
	float timeUpCounter = 0.0f;
	float timeLimit =4.0f;
	int end = 0;

	Vector3 worldPos;
	SharedPtr<Node> rootNode;
	WeakPtr<Scene> scene_;
};