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
		return NewtonToUrhoVec3(newtonBody_->CalculateAngularMomentum());
	}

	void NewtonRigidBody::SetLinearVelocity(const Vector3& worldVelocity, bool useForces)
    {
        if (newtonBody_ && !physicsWorld_->isUpdating_)
        {
            Activate();
            if (useForces)
            {
	            const ndVector curWorldVel = newtonBody_->GetVelocity();


	            const ndVector worldVel = UrhoToNewton((worldVelocity)) - curWorldVel;
                
				Vector3 worldPos = GetWorldPosition();

				//#TODO check timeStep
				newtonBody_->GetAsBodyDynamic()->ApplyImpulsePair(worldVel,ndVector(), physicsWorld_->timeStepAvg_*GetScene()->GetTimeScale());
			
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
				newtonBody_->SetLinearDamping(linearDampeningInternal_);
            }
        }
    }

    void NewtonRigidBody::SetInternalAngularDamping(float angularDamping)
    {
        angularDampeningInternal_ = Vector3(angularDamping, angularDamping, angularDamping);
        if (newtonBody_ && !physicsWorld_->isUpdating_)
        {
			newtonBody_->SetAngularDamping(UrhoToNewton(angularDampeningInternal_));
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
				newtonBody_->SetAutoSleep(autoSleep_);
            }
        }
    }

    bool NewtonRigidBody::GetAwake() const
    {
		if (newtonBody_)
			return newtonBody_->GetAutoSleep();
        else
            return false;
    }

    void NewtonRigidBody::Activate()
    {
        if (newtonBody_)
        {
			newtonBody_->SetSleepState(false);
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
			newtonBody_->SetSleepState(true);
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

    void NewtonRigidBody::DrawDebugGeometry(DebugRenderer* debug, bool depthTest, bool showAABB /*= true*/, bool showCollisionMesh /*= true*/, bool showCenterOfMass /*= true*/, bool showContactForces /*= true*/, bool showBodyFrame /*= true*/)
    {
        Component::DrawDebugGeometry(debug, depthTest);
        if (newtonBody_ && GetEffectiveNewtonShape()) {

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
                    debug->AddBoundingBox(box, Color::YELLOW, depthTest, false);

            }
            if (showCollisionMesh) 
				NewtonDebug_BodyDrawCollision(physicsWorld_, newtonBody_, debug, depthTest);

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
               
				newtonBody_->GetMassMatrix(Ixx, Iyy, Izz, mass);

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
            physicsWorld_->addToFreeQueue(newtonBody_);
            newtonBody_ = nullptr;
        }


        //also free the compound collision if there is one
        if (effectiveCollision_)
        {
            physicsWorld_->addToFreeQueue(effectiveCollision_);
            effectiveCollision_ = nullptr;
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
		physicsWorld_->GetNewtonWorld()->AddBody(newtonBody_);


        for (int densityPass = 1; densityPass >= 0; densityPass--)
        {

            if (!enabledCollisionShapes.empty())
            {

                ndShapeInstance resolvedCollision = nullptr;


                if (compoundNeeded) 
				{
					if (sceneRootBodyMode_)
					{
						//effectiveCollision_ = NewtonCreateSceneCollision(physicsWorld_->GetNewtonWorld(), 0);//internally the same as a regular compound with some flags enabled..
						effectiveCollision_ = ndShapeInstance(new ndShapeCompound());
						physicsWorld_->GetNewtonWorld()->GetSentinelBody()->SetCollisionShape(effectiveCollision_);
					}
                    else
                    {
						effectiveCollision_ = ndShapeInstance(new ndShapeCompound());
						physicsWorld_->GetNewtonWorld()->GetSentinelBody()->SetCollisionShape(effectiveCollision_);

                    }
                }
                float accumMass = 0.0f;

                NewtonCollisionShape* firstCollisionShape = nullptr;
                for (NewtonCollisionShape* colComp : enabledCollisionShapes)
                {
                    if (firstCollisionShape == nullptr)
                        firstCollisionShape = colComp;

                    //for each sub collision in the colComp
                    const ndShapeInstance* rootCollision = colComp->GetNewtonShape();
					
					
                    void* curSubNode = NewtonCompoundCollisionGetFirstNode((NewtonCollision*)rootCollision);
					

                    NewtonCollision* curSubCollision = nullptr;
                    if (curSubNode)
                        curSubCollision = NewtonCompoundCollisionGetCollisionFromNode((NewtonCollision*)rootCollision, curSubNode);
                    else
                        curSubCollision = (NewtonCollision*)rootCollision;

                    while (curSubCollision)
                    {
                        ndShapeInstance curCollisionInstance = NewtonCollisionCreateInstance(curSubCollision);
                        curSubNode = NewtonCompoundCollisionGetNextNode((NewtonCollision*)rootCollision, curSubNode);//advance
                        if (curSubNode)
                            curSubCollision = NewtonCompoundCollisionGetCollisionFromNode((NewtonCollision*)rootCollision, curSubNode);
                        else
                            curSubCollision = nullptr;


                        Quaternion colPhysworldRot = colComp->GetWorldRotation();
                        Quaternion thisNodeWorldRot = node_->GetWorldRotation();
                        Quaternion colRotLocalToThisNode = thisNodeWorldRot.Inverse() * colPhysworldRot;

                        //compute the relative vector from root node to collision
                        Vector3 relativePos = (node_->GetRotation().Inverse()*(colComp->GetWorldPosition() - node_->GetWorldPosition()));

                        //form final local matrix with physics world scaling applied.
                        Matrix3x4 nodeWorldNoScale(node_->GetWorldTransform().Translation(), node_->GetWorldTransform().Rotation(), 1.0f);
                        Matrix3x4 colWorldNoScale(colComp->GetWorldTransform().Translation(), colComp->GetWorldTransform().Rotation(), 1.0f);


                        Matrix3x4 finalLocal = nodeWorldNoScale.Inverse() * colWorldNoScale;

                        ndMatrix localTransform = UrhoToNewton(Matrix3x4((finalLocal.Translation()), colRotLocalToThisNode, 1.0f));


                        //now determine scale to apply around the center of each sub shape.
                        Vector3 scale = Vector3::ONE;
                        if (colComp->GetInheritNodeScale())
                        {
                            scale = colComp->GetRotationOffset().Inverse() * colComp->GetNode()->GetWorldScale();
                            scale = Vector3(Abs(scale.x_), Abs(scale.y_), Abs(scale.z_));
                        }
                        Vector3 shapeScale = colComp->GetScaleFactor();

                        scale = scale * shapeScale;

                        //URHO3D_LOGINFO("Shape Scale: " + String(shapeScale));
                       // URHO3D_LOGINFO("Scale: " + String(scale));

						ndVector existingLocalScale = curCollisionInstance.GetScale();

                        //URHO3D_LOGINFO("existingLocalScale from collision shape: " + String(NewtonToUrhoVec3(existingLocalScale)));


                        float densityScaleFactor = 1.0f;
                        //if we are in the first pass - scale the sub collision by the density.  so when we calculate the inertia matrix it will reflect the density of sub shapes.
                        //on the 2nd (final pass) - scale as normal.
                        if (densityPass)
                            densityScaleFactor = colComp->GetDensity()/smallestDensity;

                        //URHO3D_LOGINFO("densityScaleFactor: " + String(densityScaleFactor));

                        Vector3 finalCollisionScale = Vector3(densityScaleFactor * scale.x_*existingLocalScale.m_x,
                            densityScaleFactor*scale.y_*existingLocalScale.m_y,
                            densityScaleFactor*scale.z_*existingLocalScale.m_z);

                        //URHO3D_LOGINFO("finalCollisionScale: " + String(finalCollisionScale));

						finalCollisionScale = NewtonToUrhoVec3(curCollisionInstance.GetScale());


                        //take into account existing local matrix of the newton collision shape.
						ndMatrix existingLocalMatrix = curCollisionInstance.GetLocalMatrix();


                        Vector3 subLocalPos = NewtonToUrhoVec3(existingLocalMatrix.m_posit);
                        subLocalPos = (subLocalPos * Vector3(scale.x_*existingLocalScale.m_x, scale.y_*existingLocalScale.m_y, scale.z_*existingLocalScale.m_z));
                        subLocalPos = colComp->GetRotationOffset() * subLocalPos;
                        existingLocalMatrix.m_posit = UrhoToNewton(subLocalPos);


                        localTransform = existingLocalMatrix * localTransform;
						curCollisionInstance.SetLocalMatrix(localTransform);//set the collision matrix

                        //calculate volume
						float vol = curCollisionInstance.GetVolume();

                        accumMass += vol * colComp->GetDensity();


                        //end adding current shape.
                        if (compoundNeeded) {

                            if (sceneRootBodyMode_)
                                NewtonSceneCollisionAddSubCollision(effectiveCollision_, curCollisionInstance);
                            else
                                NewtonCompoundCollisionAddSubCollision(effectiveCollision_, curCollisionInstance);
                        }
                        else
                            resolvedCollision = curCollisionInstance;
                    }
                }
                if (compoundNeeded) {

                    NewtonCompoundCollisionEndAddRemove(effectiveCollision_);
                    resolvedCollision = effectiveCollision_;
                }

                effectiveCollision_ = resolvedCollision;

               

				newtonBody_->SetCollisionShape(resolvedCollision);

                mass_ = accumMass * massScale_;
                if (sceneRootBodyMode_)
                    mass_ = 0;

                if (densityPass) {

                    float vol = resolvedCollision.GetVolume();


					newtonBody_->SetMassMatrix(mass, finalInertia);
					

                    //save the inertia matrix for 2nd pass.
					finalInertia = newtonBody_->CalculateInertiaMatrix();

                    if (useInertiaHack_) {
                        //URHO3D_LOGINFO("Final Inertia Matrix (PreHack): " + String(NewtonToUrhoMat4(finalInertia)) + " Mass: " + String(mass_) + " Volume: " + String(vol));

                        //hack the inertia so that small values cant be too small.
                        float maxI = -M_LARGE_VALUE;
                        float minI = M_LARGE_VALUE;
                        float minFactorDiff = 10.0f;
                        for (int r = 0; r < 3; r++) {
                            if (finalInertia[r][r] > maxI)
                                maxI = finalInertia[r][r];

                            if (finalInertia[r][r] < minI && finalInertia[r][r] > 0.0f)
                                minI = finalInertia[r][r];

                        }
                        float midI = (minI + maxI)*0.5f;
                        for (int r = 0; r < 3; r++) {

                            if (finalInertia[r][r] > midI)
                                finalInertia[r][r] = maxI;
                            else
                                finalInertia[r][r] = maxI / minFactorDiff;
                        }


                       //URHO3D_LOGINFO("Hacked Inertia Matrix: " + NewtonToUrhoMat4(finalInertia).ToString() + " Mass: " + ea::to_string(mass_) + " Volume: " + ea::to_string(vol));
                    }


					finalCenterOfMass = newtonBody_->GetCentreOfMass();
                }
            }
        }

		//take final steps.


		//move resolvedCollision so that the COM matches the body frame, and calculate inertia using that, then move collision back....
		//#todo not sure why NewtonBodySetMassProperties is not doing this in the first place
	    {
			ndMatrix finalCollisionMatrix = effectiveCollision_.GetLocalMatrix();

			ndMatrix tmpCollisionMatrix = finalCollisionMatrix;
			tmpCollisionMatrix.m_posit -= finalCenterOfMass;

			effectiveCollision_.SetLocalMatrix(tmpCollisionMatrix);

			newtonBody_->SetMassMatrix(mass_, effectiveCollision_);

			finalInertia = newtonBody_->CalculateInertiaMatrix();

			effectiveCollision_.SetLocalMatrix(finalCollisionMatrix);

			newtonBody_->SetMassMatrix(mass_, effectiveCollision_);

		}

		newtonBody_->SetMassMatrix(mass_, finalInertia);

        //URHO3D_LOGINFO("Final Inertia Matrix: " + NewtonToUrhoMat4(finalInertia).ToString() + " Mass: " + ea::to_string(mass_));

      

		//if we are building the scene body - set the COM to (0,0,0)
		if (GetIsSceneRootBody()) {
			
			finalCenterOfMass = ndVector(0, 0, 0, 1);
		}

		//always keep reference to the calculated COM
		centerOfMassCalculated_ = NewtonToUrhoVec3(finalCenterOfMass);
		
		//resolve centerOfMassEffective_
		if (useCOMOffsetOverride_)
			finalCenterOfMass = UrhoToNewton(localCOMOffsetOverride_);

		

		centerOfMassEffective_ = NewtonToUrhoVec3(finalCenterOfMass);


		newtonBody_->SetCentreOfMass(UrhoToNewton(centerOfMassEffective_));

		newtonBody_->GetCollisionShape().SetCollisionMode(continuousCollision_);


        //ensure newton damping is 0 because we apply our own as a force.
		newtonBody_->SetLinearDamping(linearDampeningInternal_);

		newtonBody_->SetAngularDamping(UrhoToNewton(angularDampeningInternal_));

        //set auto sleep mode.
		newtonBody_->SetAutoSleep(autoSleep_);


        //assign callbacks
		newtonBody_->SetNotifyCallback();




        //finally move the body.
        SetWorldTransformToNode();
        lastSetNodeWorldTransform_ = node_->GetWorldTransform();



		if (mass_ > 0.0f) {
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
            if (col->IsEnabledEffective() && col->GetNewtonShape())
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
                physicsWorld_->WaitForUpdateFinished();
                physicsWorld_->removeRigidBody(this);
            }




            freeBody();

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
                //RebuildPhysicsNodeTree(oldParent);
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
				newtonBody_->SetSleepState(nextSleepState_);
                nextSleepStateNeeded_ = false;
            }
        }


    }

    void NewtonRigidBody::applyDefferedProperties()
    {
        if (!newtonBody_)
            return;

		if (newtonBody_->GetLinearDamping() != linearDampeningInternal_)
			newtonBody_->SetLinearDamping(linearDampeningInternal_);


		ndVector angularDamping = newtonBody_->GetAngularDamping();

		if (NewtonToUrhoVec3(angularDamping) != angularDampeningInternal_)
			newtonBody_->SetAngularDamping(UrhoToNewton(angularDampeningInternal_));


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
			newtonBody_->AddImpulse(UrhoToNewton((targetVelocity)), UrhoToNewton(node_->LocalToWorld(localPosition)), physicsWorld_->timeStepAvg_);
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
		return newtonBody_->GetCollisionShape();

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
			ndVector dAcc = newtonBody_->GetAccel();
            const Vector3 acc = (NewtonToUrhoVec3(dAcc));
            return acc;
        }
        else
            return Vector3::ZERO;
    }

	void NewtonRigidBody::GetConnectedContraints(ea::vector<NewtonConstraint*>& contraints)
    {
        contraints.clear();
        for (auto i = connectedConstraints_.begin(); i != connectedConstraints_.end(); ++i)
        {
            contraints.push_back(*i);
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

			ndVector v = ndVector(0, 0, 0);
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
