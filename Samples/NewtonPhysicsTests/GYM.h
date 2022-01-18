#pragma once


#include <Urho3D/Core/Object.h>
#include <Urho3D/SystemUI/SystemUI.h>
#include <Urho3D/IO/ArchiveSerialization.h>
#include <Urho3D/Graphics/DebugRenderer.h>
#include <VisualDebugger.h>
#include <ImGui/implot.h>
using namespace Urho3D;

class GYM : public Object
{
public:
	URHO3D_OBJECT(GYM, Object);

	explicit GYM(Context* context) : Object(context)
	{

	}

	struct State {
		float val;
		float max;
		float min;
	};

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
		episodeRewards.push_back(episodeReward);
		episodeReward = 0.0f;
		
	}

	virtual void PostReset()
	{
		ResizeVectors();
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
		stateCounter = 0;
		rewardPartCounter = 0;
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
		episodeReward += reward;
	}
	virtual void DrawUIStats(float timeStep)
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

		ui::Text("Episode Reward: %f", episodeReward);
		ui::Text("End: %d", end);
		ui::End();





		ui::Begin("Frame Stats");
		ui::Text("Time Step: %f", timeStep);

		ui::End();


		ui::Begin("Episode Rewards");
		if (ImPlot::BeginPlot("Reward"))
		{
			if(episodeRewards.size()>1)
				ImPlot::PlotLine<float>("Reward", &episodeRewards[1], episodeRewards.size()-1);

			ImPlot::EndPlot();
		}
		ui::End();

	}

	virtual void DrawDebugGeometry(DebugRenderer* debugRenderer)
	{

	}

	int SetNextState(float val) {

		if (stateVec.size() <= stateCounter)
		{
			stateVec.push_back(val);
			stateVec_1.push_back(0.0f);
		}
		else
		{
			stateVec[stateCounter] = val;
		}

		stateCounter++;
		return stateCounter - 1;
	}

	void BuildStateDerivatives(float timeStep) {
		for (int i = 0; i < stateCounter; i++)
		{
			float derivativeVal = (stateVec[i] - stateVec_1[i]) / timeStep;
			if (stateVec.size() <= stateCounter + i)
			{
				stateVec.push_back(derivativeVal);
			}
			else
			{
				stateVec[i + stateCounter] = derivativeVal;
			}
		}
		stateCounter *= 2;
	}


	int SetNextRewardPart(float val)
	{
		if (rewardParts.size() <= rewardPartCounter)
		{
			rewardParts.push_back(val);
		}
		else
		{
			rewardParts[rewardPartCounter] = val;
		}
		rewardPartCounter++;
		return rewardPartCounter - 1;
	}



	int stateCounter = 0;
	int rewardPartCounter = 0;

	ea::vector<float> actionVec;
	ea::vector<float> stateVec;

	ea::vector<float> actionVec_1;
	ea::vector<float> stateVec_1;

	ea::vector<float> rewardParts;

	float reward;
	float episodeReward=0.0f;
	ea::vector<float> episodeRewards;
	float timeUpCounter = 0.0f;
	float timeLimit =30.0f;
	int end = 0;

	Vector3 worldPos;
	SharedPtr<Node> rootNode;
	SharedPtr<Node> orbitNode;
	WeakPtr<Scene> scene_;
};