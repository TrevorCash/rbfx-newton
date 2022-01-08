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
#include "Urho3D/Core/Context.h"
#include "Urho3D/Core/Mutex.h"
#include "Urho3D/Core/Profiler.h"
#include "Urho3D/Graphics/DebugRenderer.h"
#include "Urho3D/Graphics/Model.h"
#include "Urho3D/Core/Context.h"
#include "Urho3D/Core/CoreEvents.h"
#include "Urho3D/Core/Object.h"
#include "Urho3D/Scene/Component.h"
#include "Urho3D/Scene/Scene.h"
#include "Urho3D/Scene/Node.h"
#include "Urho3D/Math/Sphere.h"
#include "Urho3D/Core/Thread.h"
#include "Urho3D/Engine/Engine.h"
#include "Urho3D/Math/Ray.h"
#include "Urho3D/Scene/SceneEvents.h"
#include "Urho3D/Graphics/DebugRenderer.h"
#include "Urho3D/Core/Profiler.h"
#include "Urho3D/IO/Log.h"


#include "UrhoNewtonApi.h"
#include "NewtonPhysicsWorld.h"
#include "NewtonCollisionShape.h"
#include "NewtonCollisionShapesDerived.h"
#include "NewtonRigidBody.h"
#include "UrhoNewtonConversions.h"
#include "NewtonConstraint.h"
#include "NewtonFixedDistanceConstraint.h"
#include "NewtonPhysicsEvents.h"
#include "NewtonBallAndSocketConstraint.h"
#include "NewtonKinematicsJoint.h"
#include "NewtonDebugDrawing.h"
#include "NewtonFullyFixedConstraint.h"
#include "NewtonHingeConstraint.h"
#include "NewtonSliderConstraint.h"
#include "Newton6DOFConstraint.h"





#include "EASTL/sort.h"
#include "NewtonGearConstraint.h"


namespace Urho3D {


    static const Vector3 DEFAULT_GRAVITY = Vector3(0.0f, -9.81f, 0.0f);

    NewtonPhysicsWorld::NewtonPhysicsWorld(Context* context) : Component(context)
    {
#ifdef URHO3D_STATIC
		Thread::SetMainThread();
#endif

        SubscribeToEvent(E_SCENESUBSYSTEMUPDATE, URHO3D_HANDLER(NewtonPhysicsWorld, HandleSceneUpdate));


    }

    NewtonPhysicsWorld::~NewtonPhysicsWorld()
    {
        freeWorld();
    }

    void NewtonPhysicsWorld::RegisterObject(Context* context)
    {
        context->RegisterFactory<NewtonPhysicsWorld>(DEF_PHYSICS_CATEGORY.c_str());
        URHO3D_COPY_BASE_ATTRIBUTES(Component);
        URHO3D_ACCESSOR_ATTRIBUTE("Gravity", GetGravity, SetGravity, Vector3, DEFAULT_GRAVITY, AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("Time Scale", GetTimeScale, SetTimeScale, float, 1.0f, AM_DEFAULT);
    }




	void NewtonPhysicsWorld::SerializeNewtonWorld(eastl::string fileName)
    {
        NewtonSerializeToFile(newtonWorld_, fileName.c_str(), nullptr, nullptr);
    }



    
	eastl::string NewtonPhysicsWorld::GetSolverPluginName()
    {
        void* plugin = NewtonCurrentPlugin(newtonWorld_);
		return eastl::string(NewtonGetPluginString(newtonWorld_, plugin));
    }



    void NewtonPhysicsWorld::SetGravity(const Vector3& force)
    {
        gravity_ = force;
    }


    Urho3D::Vector3 NewtonPhysicsWorld::GetGravity() const
{
        return gravity_;
    }


    void NewtonPhysicsWorld::WaitForUpdateFinished()
    {
        NewtonWaitForUpdateToFinish(newtonWorld_);
        isUpdating_ = false;
    }


	void NewtonPhysicsWorld::SetIterationCount(int numIterations /*= 8*/)
    {
        iterationCount_ = numIterations;
        applyNewtonWorldSettings();
    }


    int NewtonPhysicsWorld::GetIterationCount() const
    {
        return iterationCount_;
    }

    void NewtonPhysicsWorld::SetSubstepFactor(int factor)
    {
        subSteps_ = factor;
        applyNewtonWorldSettings();
    }

    int NewtonPhysicsWorld::GetSubstepFactor() const
    {
        return subSteps_;
    }

    void NewtonPhysicsWorld::SetThreadCount(int numThreads)
    {
        newtonThreadCount_ = numThreads;
        applyNewtonWorldSettings();
    }

    int NewtonPhysicsWorld::GetThreadCount() const
    {
        return newtonThreadCount_;
    }

    void NewtonPhysicsWorld::DrawDebugGeometry(DebugRenderer* debug, bool depthTest)
    {
        DrawDebugGeometry(debug, true, true, true, depthTest);
    }

    void NewtonPhysicsWorld::DrawDebugGeometry(DebugRenderer* debug, bool drawConstraints, bool drawContacts, bool drawRigidBodies, bool depthTest)
    {
        URHO3D_PROFILE_FUNCTION();
        if (debug)
        {
            WaitForUpdateFinished();

			

            if (drawConstraints) {
                //draw debug geometry on constraints.
                for (NewtonConstraint* constraint : constraintList)
                {
                    constraint->DrawDebugGeometry(debug, depthTest);
                }
            }

            if (drawContacts) {

            }

            if (drawRigidBodies)
            {
                //draw debug geometry on rigid bodies.
                for (NewtonRigidBody* body : rigidBodyComponentList) {
                    body->DrawDebugGeometry(debug, depthTest);
                }

                //draw debug geometry on static scene.
                if (sceneBody_)
                    sceneBody_->DrawDebugGeometry(debug, depthTest);
            }
        }
    }

   

    void NewtonPhysicsWorld::OnSceneSet(Scene* scene)
    {
        if (scene) {
            //create the newton world
            if (newtonWorld_ == nullptr) {
				newtonWorld_ = new ndWorld();
                applyNewtonWorldSettings();



                //resolve root scene body.
                if (!sceneBody_) {
                    sceneBody_ = GetScene()->GetOrCreateComponent<NewtonRigidBody>();
                    sceneBody_->SetIsSceneRootBody(true);
                    sceneBody_->SetTemporary(true);
                }


                NewtonMaterialSetCollisionCallback(newtonWorld_, 0, 0, Newton_AABBOverlapCallback, Newton_ProcessContactsCallback);
                //NewtonMaterialSetCompoundCollisionCallback(newtonWorld_, 0, 0, Newton_AABBCompoundOverlapCallback);
                NewtonSetPostUpdateCallback(newtonWorld_, Newton_PostUpdateCallback);
                NewtonWorldSetCreateDestroyContactCallback(newtonWorld_, nullptr, Newton_DestroyContactCallback);
				

            }
        }
        else
        {
            //wait for update to finish if in async mode so we can safely clean up.
			newtonWorld_->Sync();

            freeWorld();
        }
    }

    void NewtonPhysicsWorld::addCollisionShape(NewtonCollisionShape* collision)
    {
		collisionComponentList.push_front(WeakPtr<NewtonCollisionShape>(collision));
    }

    void NewtonPhysicsWorld::removeCollisionShape(NewtonCollisionShape* collision)
    {
		collisionComponentList.erase_at(collisionComponentList.index_of(WeakPtr<NewtonCollisionShape>(collision)));
    }

    void NewtonPhysicsWorld::addRigidBody(NewtonRigidBody* body)
    {
		rigidBodyComponentList.push_front(WeakPtr<NewtonRigidBody>(body));
    }

    void NewtonPhysicsWorld::removeRigidBody(NewtonRigidBody* body)
    {

		rigidBodyComponentList.erase_at(rigidBodyComponentList.index_of(WeakPtr<NewtonRigidBody>(body)));
    }

    void NewtonPhysicsWorld::addConstraint(NewtonConstraint* constraint)
    {
		constraintList.push_front(WeakPtr<NewtonConstraint>(constraint));



    }

    void NewtonPhysicsWorld::removeConstraint(NewtonConstraint* constraint)
    {
		constraintList.erase_at(constraintList.index_of(WeakPtr<NewtonConstraint>(constraint)));
    }



    void NewtonPhysicsWorld::freeWorld()
    {
        //wait for update to finish if in async mode so we can safely clean up.
		if (newtonWorld_)
			newtonWorld_->Sync();


        //free any joints
        for (NewtonConstraint* constraint : constraintList)
        {
            constraint->freeInternal();
        }
        constraintList.clear();

        //free any collision shapes currently in the list
        for (NewtonCollisionShape* col : collisionComponentList)
        {
            col->freeInternalCollision();
        }
        collisionComponentList.clear();



        //free internal bodies for all rigid bodies.
        for (NewtonRigidBody* rgBody : rigidBodyComponentList)
        {
            rgBody->freeBody();
        }
        rigidBodyComponentList.clear();



        //free meshes in mesh cache
        newtonMeshCache_.clear();

        //free the actual memory
        freePhysicsInternals();



        //destroy newton world.
        if (newtonWorld_ != nullptr) {
			delete newtonWorld_;
            newtonWorld_ = nullptr;
        }
    }




    void NewtonPhysicsWorld::addToFreeQueue(NewtonBody* newtonBody)
    {
        freeBodyQueue_.push_front(newtonBody);
    }

    void NewtonPhysicsWorld::addToFreeQueue(dCustomJoint* newtonConstraint)
    {
        freeConstraintQueue_.push_front(newtonConstraint);
    }

    void NewtonPhysicsWorld::addToFreeQueue(NewtonCollision* newtonCollision)
    {
        freeCollisionQueue_.push_front(newtonCollision);
    }

    void NewtonPhysicsWorld::applyNewtonWorldSettings()
    {
		newtonWorld_->SetSolverIterations(iterationCount_);
		newtonWorld_->SetSubSteps(subSteps_);
		newtonWorld_->SetThreadCount(newtonThreadCount_);
    }







    



    void NewtonPhysicsWorld::HandleSceneUpdate(StringHash eventType, VariantMap& eventData)
    {


       float timeStep = eventData[SceneSubsystemUpdate::P_TIMESTEP].GetFloat();

	   timeStepAvg_ += (timeStep - timeStepAvg_)*0.5f;

		//do the update.
	   timeStepAvg_ = Clamp(timeStep, 0.0f, 1.0f / 60.0f);

	   Update(timeStepAvg_, true);
		
	   
    }



    void NewtonPhysicsWorld::Update(float timestep, bool isRootUpdate)
    {

        URHO3D_PROFILE_FUNCTION();
        float physicsTimeStep = timestep*GetScene()->GetTimeScale()*timeScale_;

 

		//URHO3D_LOGINFO("updating");

        if (simulationStarted_) {
            URHO3D_PROFILE("Wait For ASync Update To finish.");
            
            NewtonWaitForUpdateToFinish(newtonWorld_);
            isUpdating_ = false;

            // Send post-step event
            VariantMap& eventData = GetEventDataMap();
            eventData[NewtonPhysicsPostStep::P_WORLD] = this;
            eventData[NewtonPhysicsPostStep::P_TIMESTEP] = physicsTimeStep;
            SendEvent(E_NEWTON_PHYSICSPOSTSTEP, eventData);

			if (stepsRemaining_ > 0)
				stepsRemaining_--;
        }
        
		if (!((stepsRemaining_ > 0) || (stepsRemaining_ <= -1)))
			return;


        if (simulationStarted_) {

            {
                URHO3D_PROFILE("Apply Node Transforms");

                {
                    URHO3D_PROFILE("Rigid Body Order Pre Sort");
                    //sort the rigidBodyComponentList by scene depth.
                    if (rigidBodyListNeedsSorted) {

						eastl::sort(rigidBodyComponentList.begin(), rigidBodyComponentList.end(), RigidBodySceneDepthCompare);
                        rigidBodyListNeedsSorted = false;
                    }
                }
            }
        }





        //rebuild stuff.
        rebuildDirtyPhysicsComponents();

        freePhysicsInternals();




        
        for (NewtonRigidBody* rigBody : rigidBodyComponentList)
        {
            if (rigBody == sceneBody_)
                continue;



            //if the node local transform has been changed since last update - move the body.
            //this is done so that a change to the scene graph will have a resulting change in the rigid body's transformation.
            //if contraints are connected dont do this.
            if (rigBody->node_->GetWorldTransform() != rigBody->lastSetNodeWorldTransform_ && (rigBody->connectedConstraints_.size() == 0)) {


                rigBody->SetWorldTransformToNode();
                rigBody->MarkInternalTransformDirty(true);

                if (rigBody->isKinematic_) {
                    //get translational matrix and apply it as velocity to the kinematic body.  this can be overridden if velocities of body are set in the physics pre-step event.
                    Matrix3x4 deltaTransform = rigBody->node_->GetWorldTransform().Inverse() * rigBody->lastSetNodeWorldTransform_;

                    Quaternion rotationalDelta = deltaTransform.Rotation();
                    Vector3 translationalDelta = deltaTransform.Translation();

                    rigBody->SetAngularVelocity(-rotationalDelta.EulerAngles()/physicsTimeStep);
                    rigBody->SetLinearVelocityHard(translationalDelta);
                }
            }

            //wake bodies around kinematic bodies
            if (rigBody->isKinematic_) {
                dVector p0, p1;
                NewtonBodyGetAABB(rigBody->newtonBody_, &p0[0], &p1[0]);

                //p0 = p0 - dVector(5, 5, 5);
                //p1 = p1 + dVector(5, 5, 5);

                NewtonWorldForEachBodyInAABBDo(newtonWorld_, &p0[0], &p1[0], Newton_WakeBodiesInAABBCallback, nullptr);
            }

			//save cached variables
			rigBody->lastLinearVelocity_ = rigBody->GetLinearVelocity(TS_WORLD);
			rigBody->lastAngularVelocity_ = rigBody->GetAngularVelocity(TS_WORLD);

            //apply the transform of all rigid body components to their respective nodes.
            if (rigBody->GetInternalTransformDirty()) {

                rigBody->ApplyTransformToNode();

                //if (rigBody->InterpolationWithinRestTolerance())
                rigBody->MarkInternalTransformDirty(false);
            }

		
        }




        {
            // Send pre-step event
            VariantMap& eventData = GetEventDataMap();
            eventData[NewtonPhysicsPreStep::P_WORLD] = this;
            eventData[NewtonPhysicsPreStep::P_TIMESTEP] = physicsTimeStep;
            SendEvent(E_NEWTON_PHYSICSPRESTEP, eventData);


            //use target time step to give newton constant time steps. 
            NewtonUpdateAsync(newtonWorld_, physicsTimeStep);
            isUpdating_ = true;
            simulationStarted_ = true;
        }
    }

    void NewtonPhysicsWorld::rebuildDirtyPhysicsComponents()
    {
        URHO3D_PROFILE_FUNCTION();


        //rebuild dirty collision shapes
        for (NewtonCollisionShape* colShape : collisionComponentList)
        {
            if (colShape->GetDirty()) {
                colShape->updateBuild();
                colShape->MarkDirty(false);
            }
        }
        

        //then rebuild rigid bodies if they need rebuilt (dirty) from root nodes up.
        for (NewtonRigidBody* rigBody : rigidBodyComponentList)
        {
            if (!rigBody->GetDirty())
                continue;

            rigBody->reBuildBody();

            rigBody->ApplyTransformToNode();
            rigBody->MarkDirty(false);
        }


        //rebuild constraints if they need rebuilt (dirty)
        for (NewtonConstraint* constraint : constraintList)
        {
			if (constraint->GetDirty()) {
				constraint->reEvalConstraint();
			}
        }


        
        for (NewtonRigidBody* rigBody : rigidBodyComponentList)
        {
            //apply properties
            rigBody->applyDefferedProperties();

            //apply deferred actions (like impulses/velocity sets etc.) that were waiting for a real body to be built.
            rigBody->applyDefferedActions();
        }
    }





    Urho3D::StringHash NewtonPhysicsWorld::NewtonMeshKey(eastl::string modelResourceName, int modelLodLevel, eastl::string otherData)
    {
        return modelResourceName + eastl::to_string(modelLodLevel) + otherData;
    }

    NewtonMeshObject* NewtonPhysicsWorld::GetCreateNewtonMesh(StringHash urhoNewtonMeshKey)
    {
        if (newtonMeshCache_.contains(urhoNewtonMeshKey)) {
            return newtonMeshCache_[urhoNewtonMeshKey];
        }
        else
        {
            NewtonMesh* mesh = NewtonMeshCreate(newtonWorld_);
            SharedPtr<NewtonMeshObject> meshObj = context_->CreateObject<NewtonMeshObject>();
            meshObj->mesh = mesh;
            newtonMeshCache_[urhoNewtonMeshKey] = meshObj;
            return meshObj;
        }
    }

    NewtonMeshObject* NewtonPhysicsWorld::GetNewtonMesh(StringHash urhoNewtonMeshKey)
    {
        if (newtonMeshCache_.contains(urhoNewtonMeshKey)) {
            return newtonMeshCache_[urhoNewtonMeshKey];
        }
        return nullptr;
    }

    void NewtonPhysicsWorld::freePhysicsInternals()
    {
        for (dCustomJoint* constraint : freeConstraintQueue_)
        {
            delete constraint;
        }
        freeConstraintQueue_.clear();


        for (NewtonCollision* col : freeCollisionQueue_)
        {
            NewtonDestroyCollision(col);
        }
        freeCollisionQueue_.clear();


        for (NewtonBody* body : freeBodyQueue_)
        {
            NewtonDestroyBody(body);
        }
        freeBodyQueue_.clear();




    }
 

    void PrintPhysicsRayCastIntersection(PhysicsRayCastIntersection& intersection)
    {
        URHO3D_LOGINFO("body_: " + Urho3D::ToString((void*)intersection.body_));
        URHO3D_LOGINFO("collision_: " + Urho3D::ToString((void*)intersection.collision_));
        URHO3D_LOGINFO("subCollision_: " + Urho3D::ToString((void*)intersection.subCollision_));
        URHO3D_LOGINFO("rayIntersectParameter_: " + ea::to_string(intersection.rayIntersectParameter_));
        URHO3D_LOGINFO("rigBody_: " + Urho3D::ToString((void*)intersection.rigBody_));
        URHO3D_LOGINFO("collisionShape_: " + Urho3D::ToString((void*)intersection.collisionShape_));
        URHO3D_LOGINFO("rayIntersectWorldPosition_: " + intersection.rayIntersectWorldPosition_.ToString());
        URHO3D_LOGINFO("rayIntersectWorldNormal_: " + intersection.rayIntersectWorldNormal_.ToString());
        URHO3D_LOGINFO("rayDistance_: " + eastl::to_string(intersection.rayDistance_));
        URHO3D_LOGINFO("rayOriginWorld_: " + intersection.rayOriginWorld_.ToString());
    }

    eastl::string NewtonThreadProfilerString(int threadIndex)
    {
        return ("Newton_Thread" + ea::to_string(threadIndex));
    }

    
    //called when the newton update finished.
    void Newton_PostUpdateCallback(const NewtonWorld* const world, dFloat timestep)
    {
        NewtonPhysicsWorld* physicsWorld = (NewtonPhysicsWorld*)NewtonWorldGetUserData(world);
        physicsWorld->isUpdating_ = false;
    }



	//add rigid bodies to the list as the function recurses from node to root. the last rigid body in rigidBodies is the most root. optionally include the scene as root.
    void GetRootRigidBodies(ea::vector<NewtonRigidBody*>& rigidBodies, Node* node, bool includeScene)
    {
        NewtonRigidBody* body = node->GetComponent<NewtonRigidBody>();

        if (body)
            rigidBodies.push_back(body);

        //recurse on parent
        if(node->GetParent() && ((node->GetScene() != node->GetParent()) || includeScene))
            GetRootRigidBodies(rigidBodies, node->GetParent(), includeScene);
    }





    //returns first occurring child rigid bodies from root to child nodes.  last elements of the list are the furthest away from the root. 
	URHONEWTON_API void GetNextChildRigidBodies(ea::vector<NewtonRigidBody*>& rigidBodies, Node* node)
    {

		ea::vector<Node*> immediateChildren;
        node->GetChildren(immediateChildren, false);

        for (Node* child : immediateChildren) {
            if (child->HasComponent<NewtonRigidBody>())
                rigidBodies.push_back(child->GetComponent<NewtonRigidBody>());
            else
                GetNextChildRigidBodies(rigidBodies, child);
        }
    }

    //recurses up the scene tree starting at the starting node,  continuing up every branch adding collision shapes to the array until a rigid body is encountered in which case the algorithm stops traversing that branch.
    void GetAloneCollisionShapes(ea::vector<NewtonCollisionShape*>& colShapes, Node* startingNode, bool includeStartingNodeShapes)
    {
		

        if (includeStartingNodeShapes)
        {
            startingNode->GetDerivedComponents<NewtonCollisionShape>(colShapes, false, false);
        }



		ea::vector<Node*> immediateChildren;
        startingNode->GetChildren(immediateChildren, false);

        for (Node* child : immediateChildren) {
            if (child->HasComponent<NewtonRigidBody>() && child->GetComponent<NewtonRigidBody>()->IsEnabled())
                continue;
            else
            {
                child->GetDerivedComponents<NewtonCollisionShape>(colShapes, false, false);
                GetAloneCollisionShapes(colShapes, child, false);
            }
        }
    }


	//given a group of rigid bodies, calculates an approximate overall velocity if the bodies were to be instantly fused together about a reference frame. (conservation of momentum)
	void CalculateRigidBodyGroupFusedVelocities(ea::vector<NewtonRigidBody*>& rigidBodies, Matrix3x4 worldReferenceFrame, Vector3& worldVelocity, Vector3& worldAngularVelocity)
	{

		//note we are currently not taking into account the local rotational velocities of each body.  
		//Only the linear velocities of each body converted into angular and linear velocities on the final group.

		Vector3 averageWorldVelocity = Vector3::ZERO;
		Vector3 averageWorldAngVelocity = Vector3::ZERO;

		float totalMass = 0.0f;
		for (NewtonRigidBody* body : rigidBodies)
		{
			totalMass += body->GetEffectiveMass();
			//URHO3D_LOGINFO(body->GetLinearVelocity(TS_WORLD).ToString());

			Vector3 displacement = body->GetWorldPosition() - worldReferenceFrame.Translation();
			Vector3 angularAboutOrigin = body->GetLinearVelocity(TS_WORLD).CrossProduct(displacement) / displacement.LengthSquared();

			averageWorldAngVelocity += (angularAboutOrigin )*body->GetEffectiveMass();
			averageWorldVelocity += body->GetLinearVelocity(TS_WORLD)*body->GetEffectiveMass();
		}

		worldVelocity = averageWorldVelocity/totalMass;
		worldAngularVelocity = -1.0f*averageWorldAngVelocity/totalMass;
	}

	URHONEWTON_API Matrix3 MoveInertiaMatrix(Matrix3 matrix, const float mass, const Vector3& delta)
	{
		//URHO3D_LOGINFO(ea::to_string(matrix.m00_) + "," + ea::to_string(matrix.m11_) + "," + ea::to_string(matrix.m22_));

		float Ixx = matrix.m00_ - mass * (delta.y_*delta.y_ + delta.z_*delta.z_);
		float Iyy = matrix.m11_ - mass * (delta.x_*delta.x_ + delta.z_*delta.z_);
		float Izz = matrix.m22_ - mass * (delta.x_*delta.x_ + delta.y_*delta.y_);
		Matrix3 newMatrix;
		newMatrix.m00_ = Ixx;
		newMatrix.m11_ = Iyy;
		newMatrix.m22_ = Izz;
		return newMatrix;
	}

	void RebuildPhysicsNodeTree(Node* node)
    {
        //trigger a rebuild on the root of the new tree.
		ea::vector<NewtonRigidBody*> rigBodies;
        GetRootRigidBodies(rigBodies, node, false);
        if (rigBodies.size()) {
            NewtonRigidBody* mostRootRigBody = rigBodies.back();
            if (mostRootRigBody)
                mostRootRigBody->MarkDirty(true);
        }
    }


    unsigned CollisionLayerAsBit(unsigned layer)
    {
        if (layer == 0)
            return M_MAX_UNSIGNED;

        return (0x1 << layer - 1);
    }

    void RegisterNewtonPhysicsLibrary(Context* context)
    {
        NewtonPhysicsWorld::RegisterObject(context);

        NewtonCollisionShape::RegisterObject(context);
        NewtonCollisionShape_Box::RegisterObject(context);
        NewtonCollisionShape_Sphere::RegisterObject(context);
        NewtonCollisionShape_Cylinder::RegisterObject(context);
        NewtonCollisionShape_ChamferCylinder::RegisterObject(context);
        NewtonCollisionShape_Capsule::RegisterObject(context);
        NewtonCollisionShape_Cone::RegisterObject(context);
        NewtonCollisionShape_Geometry::RegisterObject(context);
        NewtonCollisionShape_ConvexHull::RegisterObject(context);
        NewtonCollisionShape_ConvexHullCompound::RegisterObject(context);
        NewtonCollisionShape_ConvexDecompositionCompound::RegisterObject(context);
        NewtonCollisionShape_TreeCollision::RegisterObject(context);
        NewtonCollisionShape_HeightmapTerrain::RegisterObject(context);

        NewtonRigidBody::RegisterObject(context);
		//NewtonRigidBody::CollisionOverrideEntry::RegisterObject(context);



        NewtonMeshObject::RegisterObject(context);
        NewtonConstraint::RegisterObject(context);
        NewtonFixedDistanceConstraint::RegisterObject(context);
        NewtonBallAndSocketConstraint::RegisterObject(context);
        NewtonSixDofConstraint::RegisterObject(context);
        NewtonHingeConstraint::RegisterObject(context);
        NewtonSliderConstraint::RegisterObject(context);
        NewtonFullyFixedConstraint::RegisterObject(context);
        NewtonKinematicsControllerConstraint::RegisterObject(context);
		NewtonGearConstraint::RegisterObject(context);

    }












}
