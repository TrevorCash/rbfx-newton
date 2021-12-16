#pragma once


#include <Urho3D/Core/Object.h>

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
		FormResponses();
	}

	virtual void ResizeVectors(){}

	virtual void Update(float timeStep)
	{
		ApplyActionVec();
		FormResponses();

		timeUpCounter += timeStep;

		if (timeUpCounter > timeLimit)
			end = 1;
	}

	virtual void FormResponses()
	{

	}

	virtual void ApplyActionVec()
	{

	}

	ea::vector<float> actionVec;
	ea::vector<float> stateVec;
	float reward;
	float timeUpCounter = 0.0f;
	float timeLimit = 2.5f;
	int end = 0;

	Vector3 worldPos;
	WeakPtr<Node> rootNode;
	WeakPtr<Scene> scene_;
};