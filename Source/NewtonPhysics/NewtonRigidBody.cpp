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


#include "Newton.h"
#include "dMatrix.h"
#include "dQuaternion.h"
#include "dgQuaternion.h"


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
        URHO3D_ACCESSOR_ATTRIBUTE("Continuous Collision", GetContinuousCollision, SetContinuousCollision, bool, false, AM_DEFAULT);
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
        SetWorldTransform(node_->GetWorldTransform());
    }

    void NewtonRigidBody::SetWorldTransform(const Matrix3x4& transform)
    {
        if (newtonBody_ && !physicsWorld_->isUpdating_)
        {
            Activate();

            Matrix3x4 scaleLessTransform((transform.Translation()), transform.Rotation(), 1.0f);
            NewtonBodySetMatrix(newtonBody_, &UrhoToNewton(scaleLessTransform)[0][0]);

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

            dgQuaternion orientation;
            NewtonBodyGetRotation(newtonBody_, &orientation.m_x);

            Matrix3x4 transform((position), NewtonToUrhoQuat(orientation), 1.0f);
            NewtonBodySetMatrix(newtonBody_, &UrhoToNewton(transform)[0][0]);
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

            dVector pos;
            NewtonBodyGetPosition(newtonBody_, &pos[0]);

            Matrix3x4 transform(NewtonToUrhoVec3(pos), quaternion, 1.0f);
            NewtonBodySetMatrix(newtonBody_, &UrhoToNewton(transform)[0][0]);
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
			
            dMatrix bodyMatrix;
            NewtonBodyGetMatrix(newtonBody_, &bodyMatrix[0][0]);

            return (Matrix3x4(NewtonToUrhoMat4(bodyMatrix)));
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
            dVector bodyPos;
            NewtonBodyGetPosition(newtonBody_, &bodyPos[0]);

            return (NewtonToUrhoVec3(bodyPos));

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
           
            return node_->GetWorldTransform().Translation();
        }
    }
    
    Urho3D::Quaternion NewtonRigidBody::GetWorldRotation()
{ 
        if(newtonBody_ && !physicsWorld_->isUpdating_){
            dgQuaternion bodyOrientation;
            NewtonBodyGetRotation(newtonBody_, &bodyOrientation.m_x);
            return NewtonToUrhoQuat(bodyOrientation);
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

            return node_->GetWorldTransform().Rotation();
        }
    }
    
    Vector3 NewtonRigidBody::GetCenterOfMassPosition(bool scaledPhysicsWorldFrame)
    {
        if(newtonBody_)
        {
            dVector comPosition;
            NewtonBodyGetCentreOfMass(newtonBody_, &comPosition[0]);
            if(scaledPhysicsWorldFrame)
            {
                return (NewtonToUrhoVec3(comPosition));
            }
            else {
                return NewtonToUrhoVec3(comPosition);
            }
        }
        else {
            return Vector3::ZERO;
        }
    }
    
    Matrix3x4 NewtonRigidBody::GetCenterOfMassTransform(bool scaledPhysicsWorldFrame)
    {
        if(newtonBody_)
        {
            return Matrix3x4(GetCenterOfMassPosition(scaledPhysicsWorldFrame), GetWorldRotation(), 1.0f);
        }
        else {
            return Matrix3x4::IDENTITY;
        }
    }
    
    
    
    
    
	Urho3D::Vector3 NewtonRigidBody::GetAngularMomentum() const
	{
		dMatrix inertiaMatrix;
		NewtonBodyGetInertiaMatrix(newtonBody_, &inertiaMatrix[0][0]);
		Vector3 angularVelocity = GetAngularVelocity(TS_WORLD);


		return Vector3(inertiaMatrix[0][0]*angularVelocity.x_ + inertiaMatrix[0][1] * angularVelocity.y_ + inertiaMatrix[0][2] * angularVelocity.z_,
			inertiaMatrix[1][0] * angularVelocity.x_ + inertiaMatrix[1][1] * angularVelocity.y_ + inertiaMatrix[1][2] * angularVelocity.z_,
			inertiaMatrix[2][0] * angularVelocity.x_ + inertiaMatrix[2][1] * angularVelocity.y_ + inertiaMatrix[2][2] * angularVelocity.z_
			);
	}

	void NewtonRigidBody::SetLinearVelocity(const Vector3& worldVelocity, bool useForces)
    {
        if (newtonBody_ && !physicsWorld_->isUpdating_)
        {
            Activate();
            if (useForces)
            {
                dVector curWorldVel;
                NewtonBodyGetVelocity(newtonBody_, &curWorldVel[0]);

                dVector worldVel = UrhoToNewton((worldVelocity)) - curWorldVel;
                dVector bodyWorldPos;
                NewtonBodyGetPosition(newtonBody_, &bodyWorldPos[0]);
                NewtonBodyAddImpulse(newtonBody_, &worldVel[0], &bodyWorldPos[0], physicsWorld_->timeStepTarget_*GetScene()->GetTimeScale());
            }
            else
                NewtonBodySetVelocity(newtonBody_, &UrhoToNewton(worldVelocity)[0]);
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


            Vector3 angularVelocityRadians = angularVelocity * M_DEGTORAD;

            NewtonBodySetOmega(newtonBody_, &UrhoToNewton(angularVelocityRadians)[0]);
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
                NewtonBodySetLinearDamping(newtonBody_, linearDampeningInternal_);
            }
        }
    }

    void NewtonRigidBody::SetInternalAngularDamping(float angularDamping)
    {
        angularDampeningInternal_ = Vector3(angularDamping, angularDamping, angularDamping);
        if (newtonBody_ && !physicsWorld_->isUpdating_)
        {
            NewtonBodySetAngularDamping(newtonBody_, &UrhoToNewton(angularDampeningInternal_)[0]);
        }
    }


	void NewtonRigidBody::ApplyMomentumFromRigidBodyChildren(bool clearChildrenVelocities)
	{
		//this routine could be better.

		ea::vector<Node*> childBodyNodes;
		node_->GetChildrenWithComponent<NewtonRigidBody>(childBodyNodes, true);
		ea::vector<NewtonRigidBody*> childBodies;
		for (auto* nd : childBodyNodes) {
			childBodies.push_back(nd->GetComponent<NewtonRigidBody>());
		}

		Vector3 angularVelocity;
		Vector3 linearVelocity;

		CalculateRigidBodyGroupFusedVelocities(childBodies, GetWorldTransform(), linearVelocity, angularVelocity);

		//URHO3D_LOGINFO("angularVelocity: " + angularVelocity.ToString());
		//URHO3D_LOGINFO("linearVelocity: " + linearVelocity.ToString());

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

	void NewtonRigidBody::SetUseGyroscopicTorque(bool enable)
    {
        if (enableGyroTorque_ != enable)
        {
            enableGyroTorque_ = enable;

            if (newtonBody_ && !physicsWorld_->isUpdating_)
            {
                NewtonBodySetGyroscopicTorque(newtonBody_, enableGyroTorque_);
            }
        }


    }

    //void RigidBody::SetInterpolationFactor(float factor /*= 0.0f*/)
    //{
    //    interpolationFactor_ = Clamp(factor, M_EPSILON, 1.0f);
    //}



    void NewtonRigidBody::SetContinuousCollision(bool sweptCollision)
    {
        if (continuousCollision_ != sweptCollision) {
            continuousCollision_ = sweptCollision;
            if (newtonBody_) {
                NewtonBodySetContinuousCollisionMode(newtonBody_, sweptCollision);
            }
        }
    }


    void NewtonRigidBody::SetAutoSleep(bool enableAutoSleep)
    {
        if (autoSleep_ != enableAutoSleep)
        {
            autoSleep_ = enableAutoSleep;
            if (newtonBody_)
            {
                NewtonBodySetAutoSleep(newtonBody_, autoSleep_);
            }
        }
    }

    bool NewtonRigidBody::GetAwake() const
    {
        if (newtonBody_)
            return NewtonBodyGetSleepState(newtonBody_);
        else
            return false;
    }

    void NewtonRigidBody::Activate()
    {
        if (newtonBody_)
        {
            NewtonBodySetSleepState(newtonBody_, false);
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
            NewtonBodySetSleepState(newtonBody_, true);
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
        if (newtonBody_ && GetEffectiveNewtonCollision()) {
            if (showAABB )
            {
                    dMatrix matrix;
                    dVector p0(0.0f);
                    dVector p1(0.0f);

                    NewtonBodyGetMatrix(newtonBody_, &matrix[0][0]);
                    NewtonCollisionCalculateAABB(GetEffectiveNewtonCollision(), &matrix[0][0], &p0[0], &p1[0]);


                    Vector3 min = (NewtonToUrhoVec3(p0));
                    Vector3 max = (NewtonToUrhoVec3(p1));
                    BoundingBox box(min, max);
                    debug->AddBoundingBox(box, Color::YELLOW, depthTest, false);

            }
            if (showCollisionMesh) NewtonDebug_BodyDrawCollision(physicsWorld_, newtonBody_, debug, depthTest);
            if (showCenterOfMass) {


                dMatrix matrix;
                dVector com(0.0f);
                dVector p0(0.0f);
                dVector p1(0.0f);

                NewtonCollision* collision = GetEffectiveNewtonCollision();
                NewtonBodyGetCentreOfMass(newtonBody_, &com[0]);
                NewtonBodyGetMatrix(newtonBody_, &matrix[0][0]);
                NewtonCollisionCalculateAABB(collision, &matrix[0][0], &p0[0], &p1[0]);

                Vector3 aabbMin = NewtonToUrhoVec3(p0);
                Vector3 aabbMax = NewtonToUrhoVec3(p1);
                float aabbSize = (aabbMax - aabbMin).Length()*0.1f;

                dVector o(matrix.TransformVector(com));

                dVector x(o + matrix.RotateVector(dVector(1.0f, 0.0f, 0.0f, 0.0f))*aabbSize);
                debug->AddLine((Vector3((o.m_x), (o.m_y), (o.m_z))), (Vector3((x.m_x), (x.m_y), (x.m_z))), Color::RED, depthTest);


                dVector y(o + matrix.RotateVector(dVector(0.0f, 1.0f, 0.0f, 0.0f))*aabbSize);
                debug->AddLine((Vector3((o.m_x), (o.m_y), (o.m_z))), (Vector3((y.m_x), (y.m_y), (y.m_z))), Color::GREEN, depthTest);


                dVector z(o + matrix.RotateVector(dVector(0.0f, 0.0f, 1.0f, 0.0f))*aabbSize);
                debug->AddLine((Vector3((o.m_x), (o.m_y), (o.m_z))), (Vector3((z.m_x), (z.m_y), (z.m_z))), Color::BLUE, depthTest);
            }
			if (showBodyFrame)
			{
				dMatrix matrix;
				NewtonBodyGetMatrix(newtonBody_, &matrix[0][0]);
				debug->AddFrame(Matrix3x4(NewtonToUrhoMat4(matrix)), 1.0f, Color::RED, Color::GREEN, Color::BLUE, depthTest);
			}
            if (showContactForces)
            {

                dFloat mass;
                dFloat Ixx;
                dFloat Iyy;
                dFloat Izz;
                NewtonBodyGetMass(newtonBody_, &mass, &Ixx, &Iyy, &Izz);

                //draw normal forces in term of acceleration.
                //this mean that two bodies with same shape but different mass will display the same force
                if (mass > 0.0f) {
                    float scaleFactor = 0.1f / mass;
                    for (NewtonJoint* joint = NewtonBodyGetFirstContactJoint(newtonBody_); joint; joint = NewtonBodyGetNextContactJoint(newtonBody_, joint)) {
                        if (NewtonJointIsActive(joint)) {
                            for (void* contact = NewtonContactJointGetFirstContact(joint); contact; contact = NewtonContactJointGetNextContact(joint, contact)) {
                                dVector point(0.0f);
                                dVector normal(0.0f);
                                dVector tangentDir0(0.0f);
                                dVector tangentDir1(0.0f);
                                dVector contactForce(0.0f);
                                NewtonMaterial* const material = NewtonContactGetMaterial(contact);

                                NewtonMaterialGetContactForce(material, newtonBody_, &contactForce.m_x);
                                NewtonMaterialGetContactPositionAndNormal(material, newtonBody_, &point.m_x, &normal.m_x);
                                dVector normalforce(normal.Scale(contactForce.DotProduct3(normal)));
                                dVector p0(point);
                                dVector p1(point + normalforce.Scale(scaleFactor));

                                debug->AddLine((Vector3((p0.m_x), (p0.m_y), (p0.m_z))), (Vector3((p1.m_x), (p1.m_y), (p1.m_z))), Color::GRAY, depthTest);



                                // these are the components of the tangents forces at the contact point, the can be display at the contact position point.
                                NewtonMaterialGetContactTangentDirections(material, newtonBody_, &tangentDir0[0], &tangentDir1[0]);
                                dVector tangentForce1(tangentDir0.Scale((contactForce.DotProduct3(tangentDir0)) * scaleFactor));
                                dVector tangentForce2(tangentDir1.Scale((contactForce.DotProduct3(tangentDir1)) * scaleFactor));

                                p1 = point + tangentForce1.Scale(scaleFactor);
                                debug->AddLine((Vector3((p0.m_x), (p0.m_y), (p0.m_z))), (Vector3((p1.m_x), (p1.m_y), (p1.m_z))), Color::GRAY, depthTest);


                                p1 = point + tangentForce2.Scale(scaleFactor);
                                debug->AddLine((Vector3((p0.m_x), (p0.m_y), (p0.m_z))), (Vector3((p1.m_x), (p1.m_y), (p1.m_z))), Color::GRAY, depthTest);
                            }
                        }
                    }
                }
            }
        }
    }


    void NewtonRigidBody::MarkDirty(bool dirty)
    {
        needsRebuilt_ = dirty;

		////mark constraints dirty too
		//for (NewtonConstraint* constraint : connectedConstraints_)
		//{
		//	constraint->MarkDirty(true);
		//}
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
            NewtonBodySetUserData(newtonBody_, nullptr);
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
        dMatrix finalInertia;
        dVector finalCenterOfMass;
        dMatrix identity = dGetIdentityMatrix();


        if (!IsEnabledEffective())
            return;


        ea::vector<NewtonCollisionShape*> enabledCollisionShapes;
        updateChildCollisionShapes(enabledCollisionShapes);





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

            if(enabledCollisionShapes.size() > 1)
                newtonBody_ = NewtonCreateAsymetricDynamicBody(physicsWorld_->GetNewtonWorld(), nullptr, &identity[0][0]);
            else
                newtonBody_ = NewtonCreateDynamicBody(physicsWorld_->GetNewtonWorld(), nullptr, &identity[0][0]);
        }
        else
            newtonBody_ = NewtonCreateKinematicBody(physicsWorld_->GetNewtonWorld(), nullptr, &identity[0][0]);


        for (int densityPass = 1; densityPass >= 0; densityPass--)
        {

            if (enabledCollisionShapes.size())
            {

                NewtonCollision* resolvedCollision = nullptr;

                if (effectiveCollision_) {
                    NewtonDestroyCollision(effectiveCollision_);
                    effectiveCollision_ = nullptr;
                }

                if (compoundNeeded) {
                    if (sceneRootBodyMode_)
                        effectiveCollision_ = NewtonCreateSceneCollision(physicsWorld_->GetNewtonWorld(), 0);//internally the same as a regular compound with some flags enabled..
                    else
                        effectiveCollision_ = NewtonCreateCompoundCollision(physicsWorld_->GetNewtonWorld(), 0);

                    NewtonCompoundCollisionBeginAddRemove(effectiveCollision_);
                }
                float accumMass = 0.0f;

                NewtonCollisionShape* firstCollisionShape = nullptr;
                for (NewtonCollisionShape* colComp : enabledCollisionShapes)
                {
                    if (firstCollisionShape == nullptr)
                        firstCollisionShape = colComp;

                    //for each sub collision in the colComp
                    const NewtonCollision* rootCollision = colComp->GetNewtonCollision();

                    void* curSubNode = NewtonCompoundCollisionGetFirstNode((NewtonCollision*)rootCollision);
                    NewtonCollision* curSubCollision = nullptr;
                    if (curSubNode)
                        curSubCollision = NewtonCompoundCollisionGetCollisionFromNode((NewtonCollision*)rootCollision, curSubNode);
                    else
                        curSubCollision = (NewtonCollision*)rootCollision;

                    while (curSubCollision)
                    {
                        NewtonCollision* curCollisionInstance = NewtonCollisionCreateInstance(curSubCollision);
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

                        dMatrix localTransform = UrhoToNewton(Matrix3x4((finalLocal.Translation()), colRotLocalToThisNode, 1.0f));


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

                        dVector existingLocalScale;
                        NewtonCollisionGetScale(curCollisionInstance, &existingLocalScale.m_x, &existingLocalScale.m_y, &existingLocalScale.m_z);

                        //URHO3D_LOGINFO("existingLocalScale from collision shape: " + String(NewtonToUrhoVec3(existingLocalScale)));


                        float densityScaleFactor = 1.0f;
                        //if we are in the first pass - scale the sub collision by the density.  so when we calculate the inertia matrix it will reflect the density of sub shapes.
                        //on the 2nd (final pass - scale as normal).
                        if (densityPass)
                            densityScaleFactor = colComp->GetDensity()/smallestDensity;

                        //URHO3D_LOGINFO("densityScaleFactor: " + String(densityScaleFactor));

                        Vector3 finalCollisionScale = Vector3(densityScaleFactor * scale.x_*existingLocalScale.m_x,
                            densityScaleFactor*scale.y_*existingLocalScale.m_y,
                            densityScaleFactor*scale.z_*existingLocalScale.m_z);

                        //URHO3D_LOGINFO("finalCollisionScale: " + String(finalCollisionScale));


                        NewtonCollisionSetScale(curCollisionInstance, finalCollisionScale.x_, finalCollisionScale.y_, finalCollisionScale.z_);



                        //take into account existing local matrix of the newton collision shape.
                        dMatrix existingLocalMatrix;
                        NewtonCollisionGetMatrix(curCollisionInstance, &existingLocalMatrix[0][0]);

                        Vector3 subLocalPos = NewtonToUrhoVec3(existingLocalMatrix.m_posit);
                        subLocalPos = (subLocalPos * Vector3(scale.x_*existingLocalScale.m_x, scale.y_*existingLocalScale.m_y, scale.z_*existingLocalScale.m_z));
                        subLocalPos = colComp->GetRotationOffset() * subLocalPos;
                        existingLocalMatrix.m_posit = UrhoToNewton(subLocalPos);


                        localTransform = existingLocalMatrix * localTransform;
                        NewtonCollisionSetMatrix(curCollisionInstance, &localTransform[0][0]);//set the collision matrix


                        //calculate volume
                        float vol = NewtonConvexCollisionCalculateVolume(curCollisionInstance);

                        accumMass += vol * colComp->GetDensity();


                        //end adding current shape.
                        if (compoundNeeded) {

                            if (sceneRootBodyMode_)
                                NewtonSceneCollisionAddSubCollision(effectiveCollision_, curCollisionInstance);
                            else
                                NewtonCompoundCollisionAddSubCollision(effectiveCollision_, curCollisionInstance);

                            NewtonDestroyCollision(curCollisionInstance);//free the temp collision that was used to build the compound.
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



               
                NewtonBodySetCollision(newtonBody_, resolvedCollision);


                mass_ = accumMass * massScale_;
                if (sceneRootBodyMode_)
                    mass_ = 0;

                if (densityPass) {

                    float vol = NewtonConvexCollisionCalculateVolume(resolvedCollision);


                    NewtonBodySetMassProperties(newtonBody_, mass_, resolvedCollision);

                    //save the inertia matrix for 2nd pass.
                    NewtonBodyGetInertiaMatrix(newtonBody_, &finalInertia[0][0]);


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


                       // URHO3D_LOGINFO("Final Inertia Matrix: " + String(NewtonToUrhoMat4(finalInertia)) + " Mass: " + String(mass_) + " Volume: " + String(vol));
                    }
                    NewtonBodyGetCentreOfMass(newtonBody_, &finalCenterOfMass[0]);
                }
            }
        }

        
        Matrix4 inertiaMatrixUrho = NewtonToUrhoMat4(finalInertia);

        //URHO3D_LOGINFO("Final Inertia Matrix: " + String(inertiaMatrixUrho) + " Mass: " + String(mass_));

        NewtonBodySetFullMassMatrix(newtonBody_, mass_, &finalInertia[0][0]);
		

        NewtonBodySetCentreOfMass(newtonBody_, &finalCenterOfMass[0]);


        NewtonBodySetMaterialGroupID(newtonBody_, 0);

        NewtonBodySetUserData(newtonBody_, (void*)this);

        NewtonBodySetContinuousCollisionMode(newtonBody_, continuousCollision_);

        NewtonBodySetGyroscopicTorque(newtonBody_, enableGyroTorque_);

        //ensure newton damping is 0 because we apply our own as a force.
        NewtonBodySetLinearDamping(newtonBody_, linearDampeningInternal_);
        NewtonBodySetAngularDamping(newtonBody_, &UrhoToNewton(angularDampeningInternal_)[0]);

        //set auto sleep mode.
        NewtonBodySetAutoSleep(newtonBody_, autoSleep_);



        //assign callbacks
        NewtonBodySetForceAndTorqueCallback(newtonBody_, Newton_ApplyForceAndTorqueCallback);
        NewtonBodySetTransformCallback(newtonBody_, Newton_SetTransformCallback);
        NewtonBodySetDestructorCallback(newtonBody_, Newton_DestroyBodyCallback);

        //finally move the body.
        SetWorldTransformToNode();
        lastSetNodeWorldTransform_ = node_->GetWorldTransform();


		SetLinearVelocityHard(oldLinearVelocity);
		SetAngularVelocity(oldAngularVelocity);
    }






    void NewtonRigidBody::updateChildCollisionShapes(ea::vector<NewtonCollisionShape*>& enabledCollisionShapes)
    {
        //evaluate child nodes (+this node) and see if there are more collision shapes
        GetAloneCollisionShapes(collisionShapes_, node_, true);


        //filter out shapes that are not enabled.
		ea::vector<NewtonCollisionShape*> filteredList;
        for (NewtonCollisionShape* col : collisionShapes_)
        {
            if (col->IsEnabledEffective() && col->GetNewtonCollision())
                filteredList.push_back(col);
        }
        enabledCollisionShapes = filteredList;
    }

    //void RigidBody::updateInterpolatedTransform()
    //{
    //    interpolatedNodePos_ += (targetNodePos_ - interpolatedNodePos_)*interpolationFactor_;
    //    interpolatedNodeRotation_ = interpolatedNodeRotation_.Nlerp(targetNodeRotation_, interpolationFactor_, true);
    //}


    //bool RigidBody::InterpolationWithinRestTolerance()
    //{
    //    bool inTolerance = true;
    //    inTolerance &= ( (targetNodePos_ - interpolatedNodePos_).Length() < M_EPSILON );
    //    inTolerance &= ( (targetNodeRotation_ - interpolatedNodeRotation_).Angle() < M_EPSILON);

    //    return inTolerance;
    //}

    //void RigidBody::SnapInterpolation()
    //{
    //    interpolatedNodePos_ = targetNodePos_;
    //    interpolatedNodeRotation_ = targetNodeRotation_;
    //}

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
            //#todo ?
            //remove any connected constraints.
           // for (WeakPtr<NewtonConstraint> constraint : connectedConstraints_) {
			//	if(!constraint.Expired())
			//		constraint->Remove();
            //}



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
                URHO3D_LOGINFO("should not happen");
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
                NewtonBodySetSleepState(newtonBody_, nextSleepState_);
                nextSleepStateNeeded_ = false;
            }
        }


    }

    void NewtonRigidBody::applyDefferedProperties()
    {
        if (!newtonBody_)
            return;

        if (NewtonBodyGetLinearDamping(newtonBody_) != linearDampeningInternal_)
            NewtonBodySetLinearDamping(newtonBody_, linearDampeningInternal_);


        dVector angularDamping;
        NewtonBodyGetAngularDamping(newtonBody_, &angularDamping[0]);
        if (NewtonToUrhoVec3(angularDamping) != angularDampeningInternal_)
            NewtonBodySetAngularDamping(newtonBody_, &UrhoToNewton(angularDampeningInternal_)[0]);


        if (NewtonBodyGetGyroscopicTorque(newtonBody_) != int(enableGyroTorque_))
            NewtonBodySetGyroscopicTorque(newtonBody_, enableGyroTorque_);
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
        AddWorldForce(force, Vector3::ZERO);
    }

    void NewtonRigidBody::AddWorldForce(const Vector3& worldForce, const Vector3& worldPosition)
    {
        netForce_ += worldForce ; 
        AddWorldTorque((worldPosition - GetCenterOfMassPosition()).CrossProduct(worldForce));
    }

    void NewtonRigidBody::AddWorldTorque(const Vector3& torque)
    {
        netTorque_ += torque;
    }

    void NewtonRigidBody::AddLocalForce(const Vector3& force)
    {
        AddWorldForce(node_->GetWorldRotation() * force);
    }

    void NewtonRigidBody::AddLocalForce(const Vector3& localForce, const Vector3& localPosition)
    {
        AddWorldForce(node_->GetWorldRotation() * localForce, GetCenterOfMassTransform() * (localPosition));
    }

    void NewtonRigidBody::AddLocalTorque(const Vector3& torque)
    {
        AddWorldTorque(node_->GetWorldRotation() * torque);
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
            NewtonBodyAddImpulse(newtonBody_, &UrhoToNewton((targetVelocity))[0],
                &UrhoToNewton(node_->LocalToWorld(localPosition))[0], physicsWorld_->timeStepTarget_);
        }
        else
        {
            //schedule the impulse
            nextImpulseNeeded_ = true;
            nextImpulseLocalPos_ = localPosition;
            nextImpulseWorldVelocity_ = targetVelocity;
        }
        
    }

    Vector3 NewtonRigidBody::GetNetForce()
    {
        return (netForce_);
    }


    Urho3D::Vector3 NewtonRigidBody::GetNetTorque()
    {
        return (netTorque_);
    }

    NewtonCollision* NewtonRigidBody::GetEffectiveNewtonCollision() const
    {
        if (effectiveCollision_)
            return effectiveCollision_;

        return nullptr;
    }

    Urho3D::Vector3 NewtonRigidBody::GetLinearVelocity(TransformSpace space /*= TS_WORLD*/) const
	{
		Vector3 vel = Vector3::ZERO;
		if (newtonBody_) {

			dVector dVel;
			NewtonBodyGetVelocity(newtonBody_, &dVel[0]);
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
			return node_->WorldToLocal(vel);
		}
		else if (space == TS_PARENT)
		{
			return node_->GetParent()->WorldToLocal(vel);
		}

		return vel;

    }

    Urho3D::Vector3 NewtonRigidBody::GetAngularVelocity(TransformSpace space /*= TS_WORLD*/) const
    { 
		Vector3 angularVel;
		if (newtonBody_) {
			dVector dAngularVel;
			NewtonBodyGetOmega(newtonBody_, &dAngularVel[0]);
			Vector3 angularVel = (NewtonToUrhoVec3(dAngularVel));
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
			return node_->WorldToLocal(angularVel);
		}
		else if (space == TS_PARENT)
		{
			return node_->GetParent()->WorldToLocal(angularVel);
		}
		return angularVel;

    }


    Vector3 NewtonRigidBody::GetAcceleration()
    {
        if (newtonBody_) {
            dVector dAcc;
            NewtonBodyGetAcceleration(newtonBody_, &dAcc[0]);
            Vector3 acc = (NewtonToUrhoVec3(dAcc));
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



    void NewtonRigidBody::ApplyTransformToNode()
    {
        if (!newtonBody_) {
            return;
        }
            

        dVector pos;
        dQuaternion quat;
        NewtonBodyGetPosition(newtonBody_, &pos[0]);
        NewtonBodyGetRotation(newtonBody_, &quat.m_x);

        //updateInterpolatedTransform();
        //node_->SetScale(1.0f);
        //node_->SetWorldTransform(Matrix3x4::IDENTITY.Translation(), Quaternion::IDENTITY, 1.0f);
        Vector3 scenePos = (NewtonToUrhoVec3(pos));
        node_->SetWorldPosition(scenePos);
        
        node_->SetWorldRotation((NewtonToUrhoQuat(quat)).Normalized());

        lastSetNodeWorldTransform_ = node_->GetWorldTransform();


    }


    void NewtonRigidBody::GetForceAndTorque(Vector3& force, Vector3& torque)
    {
        URHO3D_PROFILE("GetForceAndTorque");

        //basic velocity damping forces
        Vector3 velocity = GetLinearVelocity(TS_WORLD);
        Vector3 linearDampingForce = -velocity.Normalized()*(velocity.LengthSquared())*linearDampening_ * mass_;

        if (linearDampingForce.Length() <= M_EPSILON)
            linearDampingForce = Vector3::ZERO;


        //basic angular damping forces
        Vector3 angularVelocity = GetAngularVelocity(TS_WORLD);
        Vector3 angularDampingTorque = -angularVelocity.Normalized()*(angularVelocity.LengthSquared())*angularDampening_ * mass_;

        if (angularVelocity.Length() <= M_EPSILON)
            angularDampingTorque = Vector3::ZERO;


        force = linearDampingForce + netForce_;
        torque = angularDampingTorque + netTorque_;

        
    }

}
