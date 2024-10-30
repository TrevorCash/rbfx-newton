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
#include "NewtonConstraint.h"
#include "NewtonDebugDrawing.h"
#include "UrhoNewtonConversions.h"
#include "NewtonPhysicsWorld.h"
#include "NewtonCollisionShape.h"
#include "NewtonRigidBody.h"
#include "NewtonModel.h"

#include "Urho3D/Core/Context.h"
#include "Urho3D/Core/Profiler.h"
#include "Urho3D/IO/Log.h"
#include "Urho3D/IO/MemoryBuffer.h"
#include "Urho3D/Core/Context.h"
#include "Urho3D/Graphics/Model.h"
#include "Urho3D/IO/Log.h"
#include "Urho3D/Scene/Scene.h"
#include "Urho3D/Scene/Node.h"
#include "Urho3D/Scene/SceneEvents.h"
#include "Urho3D/Engine/Engine.h"
#include "Urho3D/Core/Profiler.h"
#include "Urho3D/Core/Object.h"
#include "Urho3D/Resource/ResourceCache.h"
#include "Urho3D/Graphics/DebugRenderer.h"


#include "ndNewton.h"


namespace Urho3D {


	NewtonBodyNotifications::NewtonBodyNotifications() : ndBodyNotify(ndVector(ndFloat32(0.0f), ndFloat32(-9.81f), ndFloat32(0.0f), ndFloat32(1.0f)))
	{
	}

	void NewtonBodyNotifications::OnTransform(ndInt32 threadIndex, const ndMatrix& matrix)
	{

		rigidBodyComponent_->MarkInternalTransformDirty();
	}

	//void NewtonBodyNotifications::Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const
	//{

	//}

	void NewtonBodyNotifications::OnApplyExternalForce(ndInt32 threadIndex, ndFloat32 timestep)
	{
		ndBodyDynamic* const dynamicBody = GetBody()->GetAsBodyDynamic();

		Vector3 netForce = Vector3::ZERO;
		Vector3 netTorque = Vector3::ZERO;

		rigidBodyComponent_->GetForceAndTorque(netForce, netTorque);


		Vector3 gravityForce = newtonPhysicsWorld_->GetGravity() * rigidBodyComponent_->GetEffectiveMass();
		netForce += gravityForce;

		dynamicBody->SetForce(UrhoToNewton(netForce));
		dynamicBody->SetTorque(UrhoToNewton(netTorque));
	}




    NewtonRigidBody::NewtonRigidBody(Context* context) : Component(context)
    {
        SubscribeToEvent(E_NODEADDED, URHO3D_HANDLER(NewtonRigidBody, HandleNodeAdded));
        SubscribeToEvent(E_NODEREMOVED, URHO3D_HANDLER(NewtonRigidBody, HandleNodeRemoved));
    }

    NewtonRigidBody::~NewtonRigidBody()
    {
        if (nextAngularVelocityNeeded_ || nextImpulseNeeded_ || nextLinearVelocityNeeded_ || nextSleepStateNeeded_)
            URHO3D_LOGWARNING("Rigid Body Scheduled update did not get a chance to apply!  Consider saving the updates as attributes.");

        
    }

    void NewtonRigidBody::RegisterObject(Context* context)
    {
        context->RegisterFactory<NewtonRigidBody>(DEF_PHYSICS_CATEGORY.c_str());

        URHO3D_COPY_BASE_ATTRIBUTES(Component);

        
        URHO3D_ACCESSOR_ATTRIBUTE("MassScale", GetMassScale, SetMassScale, float, 1.0f, AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("Linear Velocity", GetLinearVelocity, SetLinearVelocityHard, Vector3, Vector3::ZERO, AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("Angular Velocity", GetAngularVelocity, SetAngularVelocity, Vector3, Vector3::ZERO, AM_DEFAULT);
        //URHO3D_ACCESSOR_ATTRIBUTE("Continuous Collision", GetContinuousCollision, SetContinuousCollision, bool, false, AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("Linear Damping", GetLinearDamping, SetLinearDamping, float, 0.0f, AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("Angular Damping", GetAngularDamping, SetAngularDamping, float, 0.0f, AM_DEFAULT);
        //URHO3D_ACCESSOR_ATTRIBUTE("Interpolation Factor", GetInterpolationFactor, SetInterpolationFactor, float, 1.0f, AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("Trigger Mode", GetTriggerMode, SetTriggerMode, bool, false, AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("Is Kinematic", GetIsKinematic, SetIsKinematic, bool, false, AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("Use Inertia Hack", GetUseInertiaHack, SetUseInertiaHack, bool, false, AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("Collision Layer", GetCollisionLayer, SetCollisionLayer, unsigned, DEFAULT_COLLISION_LAYER, AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("Collision Mask", GetCollisionLayerMask, SetCollisionLayerMask, unsigned, DEFAULT_COLLISION_MASK, AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("No Collide Override", GetNoCollideOverride, SetNoCollideOverride, bool, false, AM_DEFAULT);
        URHO3D_ATTRIBUTE("Collision Body Exceptions", VariantMap, collisionExceptions_, VariantMap(), AM_DEFAULT | AM_NOEDIT);
        URHO3D_ATTRIBUTE("Generate Contacts", bool, generateContacts_, true, AM_DEFAULT);
        URHO3D_ENUM_ATTRIBUTE("Collision Event Mode", collisionEventMode_, RigidBodyCollisionEventModeNames, COLLISION_START_END, AM_DEFAULT);


        URHO3D_ATTRIBUTE("Net Force", Vector3, netForce_, Vector3::ZERO, AM_DEFAULT | AM_NOEDIT);
        URHO3D_ATTRIBUTE("Net Torque", Vector3, netTorque_, Vector3::ZERO, AM_DEFAULT | AM_NOEDIT);
        URHO3D_ATTRIBUTE("Is Scene Root Body", bool, sceneRootBodyMode_, false, AM_DEFAULT | AM_NOEDIT);

        

    }


    void NewtonRigidBody::SetMassScale(float massDensityScale)
    {
        if (massScale_ != massDensityScale) {
            massScale_ = massDensityScale;



            MarkDirty(true);
        }
    }
   

    void NewtonRigidBody::SetWorldTransformToNode()
{
		if (!newtonBody_)
			return;

        SetWorldTransform(  node_->GetWorldTransform() );
    }

    void NewtonRigidBody::SetWorldTransform(const Matrix3x4& transform)
    {
        if (newtonBody_ && !physicsWorld_->isUpdating_)
        {
            Activate();

            const Matrix3x4 scaleLessTransform((transform.Translation()), transform.Rotation(), 1.0f);
			newtonBody_->SetMatrix(UrhoToNewton(scaleLessTransform));

        }
        else
        {
            nextTransformNeeded_ = true;
            nextTransform_ = Matrix3x4(transform.Translation(), transform.Rotation(), 1.0f);
        }
    }

    void NewtonRigidBody::SetWorldPosition(const Vector3& position)
    {
        if (newtonBody_ && !physicsWorld_->isUpdating_)
        {
            Activate();

            ndQuaternion orientation = newtonBody_->GetRotation();

            const Matrix3x4 transform((position), NewtonToUrhoQuat(orientation), 1.0f);

			newtonBody_->SetMatrix(UrhoToNewton(transform));
            
        }
        else
        {
            nextPositionNeeded_ = true;
            nextPosition_ = position;
        }


    }

    void NewtonRigidBody::SetWorldRotation(const Quaternion& quaternion)
    {
        if (newtonBody_ && !physicsWorld_->isUpdating_)
        {
        
		
			Activate();

			const ndMatrix matrix = newtonBody_->GetMatrix();


			const Matrix3x4 bodyMatrix = Matrix3x4(NewtonToUrhoMat4(matrix));
			const Matrix3x4 transform(bodyMatrix.Translation(), quaternion, 1.0f);
        


			newtonBody_->SetMatrix(UrhoToNewton(transform));
		
		
		}
        else
        {

            nextOrientationNeeded_ = true;
            nextOrientation_ = quaternion;
        }
    }

    Urho3D::Matrix3x4 NewtonRigidBody::GetWorldTransform()
    {
        if (newtonBody_ && !physicsWorld_->isUpdating_) {
			ndMatrix matrix = newtonBody_->GetMatrix();
			return Matrix3x4(NewtonToUrhoMat4(matrix));
		}
        else {

            //return the last transform altered by any recent calls to set transform etc..
            if (nextTransformNeeded_)
            {
                return nextTransform_;
            }
			
            Matrix3x4 transform = node_->GetWorldTransform();
            if (nextPositionNeeded_)
            {
				
                transform.SetTranslation(nextPosition_);
            }
            if (nextOrientationNeeded_)
            {
				
                transform.SetRotation(nextOrientation_.RotationMatrix());
            }
			

            return transform;
            
        }
    }

    Urho3D::Vector3 NewtonRigidBody::GetWorldPosition()
{
        if (newtonBody_ && !physicsWorld_->isUpdating_) {
            
			ndMatrix matrix = newtonBody_->GetMatrix();
			return Matrix3x4(NewtonToUrhoMat4(matrix)).Translation();

        }
        else {

            //return the last transform altered by any recent calls to set transform etc..

            if (nextTransformNeeded_)
            {
                return nextTransform_.Translation();
            }
            if (nextPositionNeeded_)
            {
                return nextPosition_;
            }
           
            return (node_->GetWorldTransform()).Translation();
        }
    }
    
    Urho3D::Quaternion NewtonRigidBody::GetWorldRotation()
{ 
        if(newtonBody_ && !physicsWorld_->isUpdating_){
			ndMatrix matrix = newtonBody_->GetMatrix();
			return Matrix3x4(NewtonToUrhoMat4(matrix)).Rotation();
        }
        else {
            //return the last transform altered by any recent calls to set transform etc..
            if (nextTransformNeeded_)
            {
                return nextTransform_.Rotation();
            }
            if (nextOrientationNeeded_)
            {
                return nextOrientation_;
            }

            return (node_->GetWorldTransform()).Rotation();
        }
    }
    
	void NewtonRigidBody::SetCenterOfMassLocalOffset(const Vector3& offset)
	{
		useCOMOffsetOverride_ = true;
		localCOMOffsetOverride_ = offset;

		if (newtonBody_ && !physicsWorld_->isUpdating_) {
			newtonBody_->SetCentreOfMass(UrhoToNewton(localCOMOffsetOverride_));

			centerOfMassEffective_ = localCOMOffsetOverride_;
		}
		else
			MarkDirty(true);
		
	}

	Urho3D::Matrix3x4 NewtonRigidBody::GetCOMWorldTransform()
	{
		if (newtonBody_) {
			const ndVector actualCOM = newtonBody_->GetCentreOfMass();

			if (NewtonToUrhoVec3(actualCOM) != centerOfMassEffective_) {
				URHO3D_LOGERROR("center of mass disagreement!");
				URHO3D_LOGERROR(NewtonToUrhoVec3(actualCOM).ToString() + " vs " + centerOfMassEffective_.ToString());
			}
		}

		return GetWorldTransform() * Matrix3x4(centerOfMassEffective_, Quaternion::IDENTITY, 1.0f);
	}

	void NewtonRigidBody::ResetCenterOfMass()
	{
		localCOMOffsetOverride_ = Vector3::ZERO;
		useCOMOffsetOverride_ = false;
		if (newtonBody_ && !physicsWorld_->isUpdating_) {
			newtonBody_->SetCentreOfMass(UrhoToNewton(centerOfMassCalculated_));
		}
		else
			MarkDirty(true);
	}



	Urho3D::Vector3 NewtonRigidBody::GetAngularMomentum() const
	{
		return NewtonToUrhoVec3(newtonBody_->GetAsBodyDynamic()->GetAsBodyKinematic()->CalculateAngularMomentum());
	}

	void NewtonRigidBody::SetLinearVelocity(const Vector3& worldVelocity, bool useForces)
    {
        if (newtonBody_ && !physicsWorld_->isUpdating_)
        {
            Activate();
            if (useForces)
            {
	            const ndVector curWorldVel = newtonBody_->GetVelocity();
	            const ndVector worldVelDelta = UrhoToNewton((worldVelocity)) - curWorldVel;

				Vector3 worldPos = GetWorldPosition();

				newtonBody_->GetAsBodyDynamic()->ApplyImpulsePair(worldVelDelta*GetEffectiveMass(),ndVector(0,0,0,0), physicsWorld_->timeStep_*GetScene()->GetTimeScale());
			
			}
			else {
				newtonBody_->SetVelocity(UrhoToNewton(worldVelocity));
			}
        }
        else
        {
            nextLinearVelocity_ = (worldVelocity);
            nextLinearVelocityUseForces_ = useForces;
            nextLinearVelocityNeeded_ = true;
        }
    }


    void NewtonRigidBody::SetLinearVelocityHard(const Vector3& worldVelocity)
    {
        SetLinearVelocity(worldVelocity, false);
    }

    void NewtonRigidBody::SetAngularVelocity(const Vector3& angularVelocity)
    {
        if (newtonBody_ && !physicsWorld_->isUpdating_)
        {
            Activate();

            const Vector3 angularVelocityRadians = angularVelocity * M_DEGTORAD;
			newtonBody_->SetOmega(UrhoToNewton(angularVelocityRadians));
        }
        else
        {
            nextAngularVelocity_ = angularVelocity;
            nextAngularVelocityNeeded_ = true;
        }
    }

    void NewtonRigidBody::SetLinearDamping(float dampingFactor)
    {
        dampingFactor = Urho3D::Clamp<float>(dampingFactor, 0.0f, dampingFactor);

        if (linearDampening_ != dampingFactor) {
            linearDampening_ = dampingFactor;
        }
    }

    void NewtonRigidBody::SetAngularDamping(float angularDamping)
    {
        angularDamping = Urho3D::Clamp(angularDamping, 0.0f, angularDamping);

        if (angularDamping != angularDamping) {
            angularDampening_ = angularDamping;
        }
    }

    void NewtonRigidBody::SetInternalLinearDamping(float damping)
    {
        if (linearDampeningInternal_ != damping) {
            linearDampeningInternal_ = damping;

            if (newtonBody_ && !physicsWorld_->isUpdating_)
            {
				newtonBody_->GetAsBodyDynamic()->SetLinearDamping(linearDampeningInternal_);
            }
        }
    }

    void NewtonRigidBody::SetInternalAngularDamping(float angularDamping)
    {
        angularDampeningInternal_ = Vector3(angularDamping, angularDamping, angularDamping);
        if (newtonBody_ && !physicsWorld_->isUpdating_)
        {
			newtonBody_->GetAsBodyDynamic()->SetAngularDamping(UrhoToNewton(angularDampeningInternal_));
        }
    }


	void NewtonRigidBody::ApplyMomentumFromRigidBodyChildren(bool clearChildrenVelocities)
	{
		//this routine could be better.

		ea::vector<Node*> childBodyNodes;
		node_->GetChildrenWithComponent<NewtonRigidBody>(childBodyNodes, true);
		ea::vector<NewtonRigidBody*> childBodies;
		for (const auto* nd : childBodyNodes) {
			childBodies.push_back(nd->GetComponent<NewtonRigidBody>());
		}

		Vector3 angularVelocity;
		Vector3 linearVelocity;

		CalculateRigidBodyGroupFusedVelocities(childBodies, GetWorldTransform(), linearVelocity, angularVelocity);


		SetLinearVelocity(linearVelocity, true);
		SetAngularVelocity(angularVelocity*100.0f);


		if (clearChildrenVelocities)
		{
			for (NewtonRigidBody* body : childBodies) {
				body->SetAngularVelocity(Vector3::ZERO);
				body->SetLinearVelocityHard(Vector3::ZERO);
			}
		}


	}


    //void RigidBody::SetInterpolationFactor(float factor /*= 0.0f*/)
    //{
    //    interpolationFactor_ = Clamp(factor, M_EPSILON, 1.0f);
    //}






    void NewtonRigidBody::SetAutoSleep(bool enableAutoSleep)
    {
        if (autoSleep_ != enableAutoSleep)
        {
            autoSleep_ = enableAutoSleep;
            if (newtonBody_)
            {
				newtonBody_->GetAsBodyDynamic()->SetAutoSleep(autoSleep_);
            }
        }
    }

    bool NewtonRigidBody::GetAwake() const
    {
		if (newtonBody_)
			return newtonBody_->GetAsBodyDynamic()->GetSleepState();
        else
            return false;
    }

    void NewtonRigidBody::Activate()
    {
        if (newtonBody_)
        {
            if(newtonBody_->GetAsBodyDynamic())
				newtonBody_->GetAsBodyDynamic()->SetSleepState(false);
        }
        else
        {
            nextSleepStateNeeded_ = true;
            nextSleepState_ = false;
        }
    }

    void NewtonRigidBody::DeActivate()
    {
        if (newtonBody_)
        {
			newtonBody_->GetAsBodyDynamic()->SetSleepState(true);
        }
        else
        {
            nextSleepStateNeeded_ = true;
            nextSleepState_ = true;
        }
    }

    void NewtonRigidBody::SetIsSceneRootBody(bool enable)
    {
        if (sceneRootBodyMode_ != enable) {
            sceneRootBodyMode_ = enable;
            MarkDirty(true);
        }
    }

    void NewtonRigidBody::OnMarkedDirty(Node* node)
    {

    }

    void NewtonRigidBody::DrawDebugGeometry(DebugRenderer* debug, bool depthTest,
        bool showAABB /*= true*/, bool showCollisionMesh /*= true*/,
        bool showCenterOfMass /*= true*/, bool showContactForces /*= true*/, bool showBodyFrame /*= true*/)
    {
        Component::DrawDebugGeometry(debug, depthTest);
        if (newtonBody_) {

			float localScale = physicsWorld_->debugScale_ * 0.5f;
            if (showAABB )
            {
                    ndMatrix matrix = newtonBody_->GetMatrix();
                    ndVector p0(0.0f);
                    ndVector p1(0.0f);

					newtonBody_->GetAABB(p0, p1);


                    Vector3 min = (NewtonToUrhoVec3(p0));
                    Vector3 max = (NewtonToUrhoVec3(p1));
                    BoundingBox box(min, max);

					//color by model if the body is part of one.
                    Color col;
					if(model_)
						col.FromUInt(reinterpret_cast<unsigned int>(static_cast<NewtonModel*>(model_)));
                    col.a_ = 1.0f;

                    debug->AddBoundingBox(box, col, depthTest, false);

            }
            if (showCollisionMesh) 
				DrawCollision(debug, depthTest);

            if (showCenterOfMass) {
				debug->AddFrame(GetCOMWorldTransform(), 1.2f*localScale, Color::MAGENTA, Color::YELLOW, Color::CYAN, depthTest);
            }
			if (showBodyFrame)
			{
				ndMatrix matrix = newtonBody_->GetMatrix();
				debug->AddFrame(Matrix3x4(NewtonToUrhoMat4(matrix)), 1.0f*localScale, Color::RED, Color::GREEN, Color::BLUE, depthTest);
			}
            if (showContactForces)
            {

                ndFloat32 mass;
                ndFloat32 Ixx;
                ndFloat32 Iyy;
                ndFloat32 Izz;
               
				newtonBody_->GetAsBodyDynamic()->GetMassMatrix(Ixx, Iyy, Izz, mass);

                //draw normal forces in term of acceleration.
                //this mean that two bodies with same shape but different mass will display the same force
                if (mass > 0.0f) {
                    float scaleFactor = 0.1f / mass;

                    //for (NewtonJoint* joint = NewtonBodyGetFirstContactJoint(newtonBody_); joint; joint = NewtonBodyGetNextContactJoint(newtonBody_, joint)) {
                    //    if (NewtonJointIsActive(joint)) {
                    //        for (void* contact = NewtonContactJointGetFirstContact(joint); contact; contact = NewtonContactJointGetNextContact(joint, contact)) {
                    //            ndVector point(0.0f);
                    //            ndVector normal(0.0f);
                    //            ndVector tangentDir0(0.0f);
                    //            ndVector tangentDir1(0.0f);
                    //            ndVector contactForce(0.0f);
                    //            NewtonMaterial* const material = NewtonContactGetMaterial(contact);

                    //            NewtonMaterialGetContactForce(material, newtonBody_, &contactForce.m_x);
                    //            NewtonMaterialGetContactPositionAndNormal(material, newtonBody_, &point.m_x, &normal.m_x);
                    //            ndVector normalforce(normal.Scale(contactForce.DotProduct3(normal)));
                    //            ndVector p0(point);
                    //            ndVector p1(point + normalforce.Scale(scaleFactor*localScale));

                    //            debug->AddLine((Vector3((p0.m_x), (p0.m_y), (p0.m_z))), (Vector3((p1.m_x), (p1.m_y), (p1.m_z))), Color::GRAY, depthTest);



                    //            // these are the components of the tangents forces at the contact point, the can be display at the contact position point.
                    //            NewtonMaterialGetContactTangentDirections(material, newtonBody_, &tangentDir0[0], &tangentDir1[0]);
                    //            ndVector tangentForce1(tangentDir0.Scale((contactForce.DotProduct3(tangentDir0)) * scaleFactor * localScale));
                    //            ndVector tangentForce2(tangentDir1.Scale((contactForce.DotProduct3(tangentDir1)) * scaleFactor * localScale));

                    //            p1 = point + tangentForce1.Scale(scaleFactor * localScale);
                    //            debug->AddLine((Vector3((p0.m_x), (p0.m_y), (p0.m_z))), (Vector3((p1.m_x), (p1.m_y), (p1.m_z))), Color::GRAY, depthTest);


                    //            p1 = point + tangentForce2.Scale(scaleFactor * localScale);
                    //            debug->AddLine((Vector3((p0.m_x), (p0.m_y), (p0.m_z))), (Vector3((p1.m_x), (p1.m_y), (p1.m_z))), Color::GRAY, depthTest);
                    //        }
                    //    }
                    //}
                }
            }
        }
    }

    void NewtonRigidBody::DrawCollision(DebugRenderer* debug, bool depthTest)
    {
	    debugRenderOptions options;
	    options.debug = debug;
	    options.depthTest = depthTest;


	    if (newtonBody_->GetAsBodyDynamic())
	    {
		    int sleepState = newtonBody_->GetAsBodyDynamic()->GetSleepState();

		    if (sleepState == 1) {
			    // indicate when body is sleeping
			    options.color = Color::BLUE;
		    }
		    else {
			    // body is active
			    options.color = Color::GREEN;
		    }
	    }
	    else if (newtonBody_->GetAsBodyKinematic())
	    {
		    options.color = Color::WHITE;
	    }


		NewtonShapeDebugNotify tmpNotify;
		tmpNotify.options = options;

		newtonBody_->GetAsBodyKinematic()->GetCollisionShape().DebugShape(newtonBody_->GetMatrix(), tmpNotify);
    }


    void NewtonRigidBody::MarkDirty(bool dirty)
    {
        needsRebuilt_ = dirty;

		if (dirty) {

			//mark all constraints connected to this body and possible bodies on child nodes as dirty so they get rebuild and re-resolved.
			ea::vector<Node*> childBodyNodes;
			node_->GetChildrenWithComponent<NewtonRigidBody>(childBodyNodes, true);
			childBodyNodes.push_back(node_);
			for (const Node* nd : childBodyNodes) {
				for (NewtonConstraint* constraint : nd->GetComponent<NewtonRigidBody>()->connectedConstraints_)
				{
					constraint->MarkDirty(true);
				}
			}
		}
    }

    void NewtonRigidBody::MarkInternalTransformDirty(bool dirty)
    {
        transformDirty_ = dirty;
    }

    bool NewtonRigidBody::GetInternalTransformDirty()
    {
        return transformDirty_;
    }


    void NewtonRigidBody::OnSetEnabled()
    {
        if (IsEnabledEffective()) {
            MarkDirty(true);//rebuild.
        }
        else
        {
            freeBody();

            //dirty constraints in case they need to resolve to parent bodies.
            for (NewtonConstraint* constraint : connectedConstraints_)
            {
				constraint->MarkDirty();
            }
        }
    }

    


	void NewtonRigidBody::calculateSceneDepth()
    {
        sceneDepth_ = 0;
        Node* curNode = node_;
        while (curNode != GetScene())
        {
            curNode = curNode->GetParent();
            sceneDepth_++;
        }
    }

    void NewtonRigidBody::freeBody()
    {
        if (newtonBody_ != nullptr) {
            physicsWorld_->removeNewtonBodyQueue_.push_back(newtonBody_);
            newtonBody_->SetNotifyCallback(nullptr);
            newtonNotifications_ = nullptr;
           
            newtonBody_ = nullptr;
        }
    }


    void NewtonRigidBody::reBuildBody()
    {
        URHO3D_PROFILE_FUNCTION();

        //save existing velocities for restoration after the rebuild
        Vector3 oldLinearVelocity = GetLinearVelocity();
        Vector3 oldAngularVelocity = GetAngularVelocity();


        freeBody();
        ndMatrix finalInertia;
        ndVector finalCenterOfMass;
		ndMatrix identity = ndGetIdentityMatrix();


        if (!IsEnabledEffective())
            return;

		//URHO3D_LOGINFO("rebuildbody..");
        ea::vector<NewtonCollisionShape*> enabledCollisionShapes;
        updateChildCollisionShapes(enabledCollisionShapes);


		//URHO3D_LOGINFO("child collision shapes: " + ea::to_string(enabledCollisionShapes.size()));


        ///determine early on if a compound is going to be needed.
        bool compoundNeeded = false;
        float smallestDensity = M_LARGE_VALUE;
        for (NewtonCollisionShape* col : enabledCollisionShapes)
        {
            if (col->IsCompound())
                compoundNeeded = true;


            if (col->GetDensity() < smallestDensity)
                smallestDensity = col->GetDensity();
        }
        compoundNeeded |= (enabledCollisionShapes.size() > 1);




        if (!isKinematic_) {

			if (enabledCollisionShapes.size() > 1)
			{
				//newtonBody_ = NewtonCreateAsymetricDynamicBody(physicsWorld_->GetNewtonWorld(), nullptr, &identity[0][0]);
				//newtonBody_ = new ndAsymetricInertiaBody();
				//physicsWorld_->GetNewtonWorld()->AddBody(newtonBody_);
				newtonBody_ = new ndBodyDynamic();
			}
			else
			{
				//newtonBody_ = NewtonCreateDynamicBody(physicsWorld_->GetNewtonWorld(), nullptr, &identity[0][0]);
				newtonBody_ = new ndBodyDynamic();
			}
        }
		else
		{
			newtonBody_ = new ndBodyKinematic();

		}

		newtonNotifications_ = new NewtonBodyNotifications();
		newtonNotifications_->rigidBodyComponent_ = this;
		newtonNotifications_->newtonPhysicsWorld_ = physicsWorld_;

		for (int densityPass = 1; densityPass >= 0; densityPass--)
		{

			effectiveCollision_ = new ndShapeCompound();


			effectiveCollision_.GetShape()->GetAsShapeCompound()->BeginAddRemove();
			float totalVolume = 0.0f;
			float totalMass = 0.0f;

			for (NewtonCollisionShape* colComp : enabledCollisionShapes)
			{

				//collect all subShapes in the colComp (normally 1, but may be more if the colComp is itself a compound).
				ea::vector<ndShapeInstance> subShapes;
				ndShapeCompound* shapeAsCompound = colComp->GetNewtonShape().GetShape()->GetAsShapeCompound();
				if (shapeAsCompound)
				{
					shapeAsCompound->BeginAddRemove();
					ndShapeCompound::ndTreeArray::Iterator it(shapeAsCompound->GetTree());
					for (it.Begin(); it; it++)
					{
						subShapes.push_back(*it.GetNode()->GetInfo()->GetShape());
					}
					shapeAsCompound->EndAddRemove();
				}
				else
				{
					ndShapeInstance shape = colComp->GetNewtonShape();


					Matrix3x4 colCompNodeToBodyNode = (node_->GetWorldTransform().Inverse() * colComp->GetNode()->GetWorldTransform());
					Matrix3x4 localMatrix = colCompNodeToBodyNode * colComp->GetOffsetMatrix();
					Matrix3x4 localMatrixPosRot = Matrix3x4(localMatrix.Translation(), localMatrix.Rotation(), 1.0f);


					float densityScale = 1.0f;

					if (densityPass)
						densityScale = colComp->GetDensity();


					shape.SetLocalMatrix(UrhoToNewton(localMatrixPosRot));//no sheer for newton..
                    Vector3 scale = colComp->GetNode()->GetScale() * colComp->GetScaleFactor() * densityScale;
                    scale = colComp->GetRotationOffset().RotationMatrix() * scale;
					shape.SetScale(UrhoToNewton(scale));


					subShapes.push_back(shape);
				}


				for (ndShapeInstance subShape : subShapes)
				{
					totalVolume += subShape.GetVolume();
					totalMass += subShape.GetVolume()*colComp->GetDensity();

					effectiveCollision_.GetShape()->GetAsShapeCompound()->AddCollision(&subShape);
				}
			}

			effectiveCollision_.GetShape()->GetAsShapeCompound()->EndAddRemove();

			mass_ = totalMass*massScale_;
			if (GetIsSceneRootBody())
				mass_ = 0.0f;



			newtonBody_->GetAsBodyKinematic()->SetCollisionShape(effectiveCollision_);
            if (newtonBody_->GetAsBodyDynamic())
            {
                newtonBody_->GetAsBodyDynamic()->SetMassMatrix(mass_, effectiveCollision_);
                //newtonBody_->GetAsBodyDynamic()->SetMassMatrix(1.0 / mass_, 1.0 / mass_, 1.0 / mass_, mass_);
                finalCenterOfMass = newtonBody_->GetAsBodyDynamic()->GetCentreOfMass();
            }
		}


		//always keep reference to the calculated COM
		centerOfMassCalculated_ = NewtonToUrhoVec3(finalCenterOfMass);
		
		//resolve centerOfMassEffective_
		if (useCOMOffsetOverride_)
			finalCenterOfMass = UrhoToNewton(localCOMOffsetOverride_);

		

		centerOfMassEffective_ = NewtonToUrhoVec3(finalCenterOfMass);


		newtonBody_->SetCentreOfMass(UrhoToNewton(centerOfMassEffective_));


        if (newtonBody_->GetAsBodyDynamic())
        {
            //ensure newton damping is 0 because we apply our own as a force.
            newtonBody_->GetAsBodyDynamic()->SetLinearDamping(linearDampeningInternal_);

            newtonBody_->GetAsBodyDynamic()->SetAngularDamping(UrhoToNewton(angularDampeningInternal_));

        }
        //set auto sleep mode.
		newtonBody_->GetAsBodyKinematic()->SetAutoSleep(autoSleep_);

		

        //assign callbacks
		newtonBody_->SetNotifyCallback(newtonNotifications_);


		physicsWorld_->GetNewtonWorld()->AddBody(newtonBody_);

        
        // move the body.
        SetWorldTransformToNode();
        lastSetNodeWorldTransform_ = node_->GetWorldTransform();


		
		if (mass_ > 0.0f) {
			//restore velocity
			SetLinearVelocityHard(oldLinearVelocity);
			SetAngularVelocity(oldAngularVelocity);
		}
		else
		{
			SetLinearVelocityHard(Vector3::ZERO);
			SetAngularVelocity(Vector3::ZERO);
		}
    }






    void NewtonRigidBody::updateChildCollisionShapes(ea::vector<NewtonCollisionShape*>& enabledCollisionShapes)
    {
        //evaluate child nodes (+this node) and see if there are more collision shapes
		collisionShapes_.clear();
        GetAloneCollisionShapes(collisionShapes_, node_, true);


        //filter out shapes that are not enabled.
		ea::vector<NewtonCollisionShape*> filteredList;
        for (NewtonCollisionShape* col : collisionShapes_)
        {
            if (col->IsEnabledEffective())
                filteredList.push_back(col);
        }
        enabledCollisionShapes = filteredList;
    }



    void NewtonRigidBody::OnNodeSet(Node* node)
    {
        if (node)
        {

            //Auto-create a physics world on the scene if it does not yet exist.
			physicsWorld_ = WeakPtr<NewtonPhysicsWorld>(GetScene()->GetOrCreateComponent<NewtonPhysicsWorld>());

            physicsWorld_->addRigidBody(this);

            node->AddListener(this);

            calculateSceneDepth();
            physicsWorld_->markRigidBodiesNeedSorted();



            prevNode_ = node;
        }
        else
        {

            if (physicsWorld_) {
                physicsWorld_->removeRigidBody(this);
            }


            prevNode_ = nullptr;
        }

    }

    void NewtonRigidBody::OnSceneSet(Scene* scene)
    {

    }
    void NewtonRigidBody::HandleNodeAdded(StringHash event, VariantMap& eventData)
    {
        Node* node = static_cast<Node*>(eventData[NodeAdded::P_NODE].GetPtr());
        Node* newParent = static_cast<Node*>(eventData[NodeRemoved::P_PARENT].GetPtr());

        if (node == node_)
        {
            RebuildPhysicsNodeTree(node);
            calculateSceneDepth();
            physicsWorld_->markRigidBodiesNeedSorted();
        }
    }

    void NewtonRigidBody::HandleNodeRemoved(StringHash event, VariantMap& eventData)
    {
        Node* node = static_cast<Node*>(eventData[NodeRemoved::P_NODE].GetPtr());
        if (node == node_)
        {
            Node* oldParent = static_cast<Node*>(eventData[NodeRemoved::P_PARENT].GetPtr());

            if (oldParent)
            {
                RebuildPhysicsNodeTree(oldParent);
            }
            else
            {
                URHO3D_LOGWARNING("NewtonRigidBody::HandleNodeRemoved: should not happen");
            }
        }
    }




    void NewtonRigidBody::applyDefferedActions()
    {
        if (nextPositionNeeded_ && !nextTransformNeeded_)
        {
            if (newtonBody_)
            {
                SetWorldPosition(nextPosition_);
                nextPositionNeeded_ = false;
            }
        }

        if (nextOrientationNeeded_ && !nextTransformNeeded_)
        {
            if (newtonBody_)
            {
                SetWorldRotation(nextOrientation_);
                nextOrientationNeeded_ = false;
            }
            
        }

        if (nextTransformNeeded_)
        {

            if (newtonBody_)
            {
                SetWorldTransform(nextTransform_);

                nextTransformNeeded_ = false;
            }
            
        }

        if (nextLinearVelocityNeeded_)
        {
            if (newtonBody_)
            {
                SetLinearVelocity(nextLinearVelocity_, nextLinearVelocityUseForces_);
                nextLinearVelocityNeeded_ = false;
            }
            
        }
        if (nextAngularVelocityNeeded_)
        {
            if (newtonBody_)
            {
                SetAngularVelocity(nextAngularVelocity_);
                nextAngularVelocityNeeded_ = false;
            }
            
        }
        if (nextImpulseNeeded_)
        {
            if (newtonBody_) {
                AddImpulse(nextImpulseLocalPos_, nextImpulseWorldVelocity_);
                nextImpulseNeeded_ = false;
            }

        }


        if (nextSleepStateNeeded_) {

            if (newtonBody_)
            {
				newtonBody_->GetAsBodyDynamic()->SetSleepState(nextSleepState_);
                nextSleepStateNeeded_ = false;
            }
        }


    }

    void NewtonRigidBody::applyDefferedProperties()
    {
        if (!newtonBody_)
            return;

        if(newtonBody_->GetAsBodyDynamic())
        {
			if (newtonBody_->GetAsBodyDynamic()->GetLinearDamping() != linearDampeningInternal_)
				newtonBody_->GetAsBodyDynamic()->SetLinearDamping(linearDampeningInternal_);


			ndVector angularDamping = newtonBody_->GetAsBodyDynamic()->GetAngularDamping();

			if (NewtonToUrhoVec3(angularDamping) != angularDampeningInternal_)
				newtonBody_->GetAsBodyDynamic()->SetAngularDamping(UrhoToNewton(angularDampeningInternal_));

        }

    }

    void NewtonRigidBody::OnNodeSetEnabled(Node* node)
    {
        if (node == node_)
        {
            if (IsEnabledEffective()) {
                MarkDirty(true);//rebuild
            }
            else
            {
                freeBody();
            }
        }
    }

    void NewtonRigidBody::AddWorldForce(const Vector3& force)
    {
        AddWorldForce(force, GetCOMWorldTransform().Translation());
    }

    void NewtonRigidBody::AddWorldForce(const Vector3& worldForce, const Vector3& worldPosition)
    {
        netForce_ += worldForce ;


        const Vector3 worldTorque = (worldPosition - GetCOMWorldTransform().Translation()).CrossProduct(worldForce);
        AddWorldTorque(worldTorque);
    }

    void NewtonRigidBody::AddWorldTorque(const Vector3& torque)
    {
        netTorque_ += torque;
		
    }

    void NewtonRigidBody::AddLocalForce(const Vector3& force)
    {
        AddWorldForce(GetWorldRotation() * force);
    }

    void NewtonRigidBody::AddLocalForce(const Vector3& localForce, const Vector3& localPosition)
    {
        AddWorldForce(GetWorldRotation() * localForce, GetWorldTransform() * (localPosition));
    }

    void NewtonRigidBody::AddLocalTorque(const Vector3& torque)
    {
        AddWorldTorque(GetWorldRotation() * torque);
    }

    void NewtonRigidBody::ResetForces()
	{
		netForce_ = Vector3(0, 0, 0);
		netTorque_ = Vector3(0, 0, 0);
    }

    void NewtonRigidBody::AddImpulse(const Vector3& localPosition, const Vector3& targetVelocity)
    {


        if (newtonBody_) {
            Activate();
			newtonBody_->GetAsBodyDynamic()->AddImpulse(UrhoToNewton((targetVelocity)), UrhoToNewton(node_->LocalToWorld(localPosition)), physicsWorld_->timeStep_);
        }
        else
        {
            //schedule the impulse
            nextImpulseNeeded_ = true;
            nextImpulseLocalPos_ = localPosition;
            nextImpulseWorldVelocity_ = targetVelocity;
        }
        
    }

    Vector3 NewtonRigidBody::GetNetWorldForce()
    {
        return (netForce_);
    }


	void NewtonRigidBody::SetNetWorldForce(Vector3 force)
	{
		netForce_ = force;
	}

	Urho3D::Vector3 NewtonRigidBody::GetNetWorldTorque()
{
        return (netTorque_);
    }

	void NewtonRigidBody::SetNetWorldTorque(Vector3 torque)
	{
		netTorque_ = torque;
	}

	ndShapeInstance& NewtonRigidBody::GetEffectiveNewtonShape() const
    {
		return newtonBody_->GetAsBodyKinematic()->GetCollisionShape();

    }

    Urho3D::Vector3 NewtonRigidBody::GetLinearVelocity(TransformSpace space /*= TS_WORLD*/) const
	{
		Vector3 vel = Vector3::ZERO;
		if (newtonBody_) {
			ndVector dVel = newtonBody_->GetVelocity();
			vel = (NewtonToUrhoVec3(dVel));
		}
		else
		{
			//use last set (cached) linear velocity.
			vel = lastLinearVelocity_;
		}
		
		if (space == TS_WORLD)
		{
			return vel;
		}
		else if (space == TS_LOCAL)
		{
			return node_->GetWorldRotation().Inverse() * vel;
		}
		else if (space == TS_PARENT)
		{
			return node_->GetParent()->GetWorldRotation().Inverse() * vel;
		}

		return vel;

    }

    Urho3D::Vector3 NewtonRigidBody::GetAngularVelocity(TransformSpace space /*= TS_WORLD*/) const
    { 
		Vector3 angularVel;
		if (newtonBody_) {
			ndVector dAngularVel = newtonBody_->GetOmega();
			angularVel = (NewtonToUrhoVec3(dAngularVel));
		}
		else
		{
			angularVel = lastAngularVelocity_;
		}


		if (space == TS_WORLD)
		{
			return angularVel;
		}
		else if (space == TS_LOCAL)
		{
			return node_->GetWorldRotation().Inverse() * angularVel;
		}
		else if (space == TS_PARENT)
		{
			return node_->GetParent()->GetWorldRotation().Inverse() * angularVel;
		}
		return angularVel;

    }


    Vector3 NewtonRigidBody::GetAcceleration()
    {
        if (newtonBody_) {
			ndVector dAcc = newtonBody_->GetAsBodyDynamic()->GetAccel();
            const Vector3 acc = (NewtonToUrhoVec3(dAcc));
            return acc;
        }
        else
            return Vector3::ZERO;
    }


	Matrix3 NewtonRigidBody::GetMassMatrix()
	{
		if (newtonBody_) {
			ndVector inertiaDiags = newtonBody_->GetAsBodyDynamic()->GetMassMatrix();
			Matrix3 inertiaMatrix;
			inertiaMatrix.m00_ = inertiaDiags.m_x;
			inertiaMatrix.m11_ = inertiaDiags.m_y;
			inertiaMatrix.m22_ = inertiaDiags.m_z;
			return inertiaMatrix;
		}
		else
			return Matrix3::IDENTITY*0.0f;
	}

	void NewtonRigidBody::GetConnectedContraints(ea::vector<NewtonConstraint*>& contraints)
    {
        contraints.clear();
        for (auto i = connectedConstraints_.begin(); i != connectedConstraints_.end(); ++i)
        {
            contraints.push_back(*i);
        }
    }
	void NewtonRigidBody::GetConnectedBodies(ea::vector<NewtonRigidBody*>& bodies)
	{
		bodies.clear();
		for (auto i = connectedConstraints_.begin(); i != connectedConstraints_.end(); ++i)
		{
			if((*i)->GetOwnBody() != this)
				bodies.push_back((*i)->GetOwnBody());

			if ((*i)->GetOtherBody() != this)
				bodies.push_back((*i)->GetOtherBody());
		}
	}

	ea::vector<NewtonConstraint*> NewtonRigidBody::GetConnectedContraints()
    {
		ea::vector<NewtonConstraint*> contraints;
        GetConnectedContraints(contraints);
        return contraints;
    }


	//applies the newtons body's body frame to the node.
    void NewtonRigidBody::ApplyTransformToNode()
    {
        if (!newtonBody_) {
            return;
        }
            

		ndVector pos = newtonBody_->GetPosition();
		ndQuaternion quat = newtonBody_->GetRotation();


        const Vector3 scenePos = NewtonToUrhoVec3(pos);

		if (scenePos.IsInf() || scenePos.IsNaN()) {
			URHO3D_LOGWARNING("Newton Body Position is Inf or Nan (trying to revert newtonbody matrix to the node's matrix");
			

			//reset the body's state
			newtonBody_->SetMatrix(UrhoToNewton(node_->GetWorldTransform()));

			ndVector v = ndVector(0, 0, 0, 0);
			newtonBody_->SetVelocity(v);
			newtonBody_->SetOmega(v);


			//verify the state has changed back to valid position
			pos = newtonBody_->GetPosition();
			quat = newtonBody_->GetRotation();

			return;
		}

        node_->SetWorldPosition(scenePos);
        node_->SetWorldRotation(NewtonToUrhoQuat(quat).Normalized());



        lastSetNodeWorldTransform_ = node_->GetWorldTransform();
    }


    void NewtonRigidBody::GetForceAndTorque(Vector3& force, Vector3& torque)
    {
        URHO3D_PROFILE("GetForceAndTorque");

        //basic velocity damping forces
        Vector3 velocity = GetLinearVelocity(TS_WORLD);

		while (velocity.Length() > M_LARGE_VALUE)//check for large value that can occasionally happen.
			velocity *= 0.5f;

        Vector3 linearDampingForce = -velocity.Normalized()*(velocity.LengthSquared())*linearDampening_ * mass_;

        if (linearDampingForce.Length() <= M_EPSILON)
            linearDampingForce = Vector3::ZERO;


        //basic angular damping forces
        const Vector3 angularVelocity = GetAngularVelocity(TS_WORLD);
        Vector3 angularDampingTorque = -angularVelocity.Normalized()*(angularVelocity.LengthSquared())*angularDampening_ * mass_;

        if (angularVelocity.Length() <= M_EPSILON)
            angularDampingTorque = Vector3::ZERO;


        force = linearDampingForce + netForce_;
        torque = angularDampingTorque + netTorque_;

        
    }




}
