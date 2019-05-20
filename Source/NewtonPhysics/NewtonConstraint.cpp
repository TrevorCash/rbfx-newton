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
#include "NewtonRigidBody.h"
#include "NewtonPhysicsWorld.h"
#include "UrhoNewtonConversions.h"
#include "NewtonDebugDrawing.h"


#include "Urho3D/Core/Context.h"
#include "Urho3D/Scene/Component.h"
#include "Urho3D/Graphics/DebugRenderer.h"
#include "Urho3D/Scene/Scene.h"
#include "Urho3D/IO/Log.h"

#include "dCustomFixDistance.h"
#include "Newton.h"



namespace Urho3D {


    const char* solveModeNames[] =
    {
        "SOLVE_MODE_JOINT_DEFAULT",
        "SOLVE_MODE_EXACT",
        "SOLVE_MODE_ITERATIVE",
        "SOLVE_MODE_KINEMATIC_LOOP",
        nullptr
    };


    NewtonConstraint::NewtonConstraint(Context* context) : Component(context)
    {

    }

    NewtonConstraint::~NewtonConstraint()
    {
    }

    void NewtonConstraint::RegisterObject(Context* context)
    {
        context->RegisterFactory<NewtonConstraint>(DEF_PHYSICS_CATEGORY.c_str());
        URHO3D_COPY_BASE_ATTRIBUTES(Component);

        URHO3D_ENUM_ACCESSOR_ATTRIBUTE("Solver Iterations", GetSolveMode, SetSolveMode, CONSTRAINT_SOLVE_MODE, solveModeNames, SOLVE_MODE_JOINT_DEFAULT, AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("Stiffness", GetStiffness, SetStiffness, float, 0.7f, AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("ForceCalculationsEnabled", GetEnableForceCalculation, SetEnableForceCalculation, bool, false, AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("Other Body ID", GetOtherBodyId, SetOtherBodyId, unsigned, 0, AM_DEFAULT | AM_COMPONENTID);

        URHO3D_ATTRIBUTE("Prev Built Own Transform", Matrix3x4, prevBuiltOwnTransform_, Matrix3x4::IDENTITY, AM_DEFAULT);
        URHO3D_ATTRIBUTE("Prev Built Other Transform", Matrix3x4, prevBuiltOtherTransform_, Matrix3x4::IDENTITY, AM_DEFAULT);
        URHO3D_ATTRIBUTE("Prev Built Own Body Transform", Matrix3x4, prevBuiltOwnBodyTransform_, Matrix3x4::IDENTITY, AM_DEFAULT);
        URHO3D_ATTRIBUTE("Prev Built Other Body Transform", Matrix3x4, prevBuiltOtherBodyTransform_, Matrix3x4::IDENTITY, AM_DEFAULT);
        URHO3D_ATTRIBUTE("Has Been Built", bool, hasBeenBuilt_, Matrix3x4::IDENTITY, AM_DEFAULT);




        URHO3D_ATTRIBUTE("Other Body Frame Position", Vector3, otherPosition_, Vector3::ZERO, AM_DEFAULT);
        URHO3D_ATTRIBUTE("Other Body Frame Rotation", Quaternion, otherRotation_, Quaternion::IDENTITY, AM_DEFAULT);
        URHO3D_ATTRIBUTE("Body Frame Position", Vector3, position_, Vector3::ZERO, AM_DEFAULT);
        URHO3D_ATTRIBUTE("Body Frame Rotation", Quaternion, rotation_, Quaternion::IDENTITY, AM_DEFAULT);


    }

    void NewtonConstraint::DrawDebugGeometry(DebugRenderer* debug, bool depthTest)
    {
        if (!IsEnabled())
            return;

        //draw 2 part line from one frame to the other. Black touching own body and gray touching other body.
        Vector3 midPoint = (GetOtherWorldFrame().Translation() + GetOwnWorldFrame().Translation())*0.5f;
        debug->AddLine(GetOwnWorldFrame().Translation(), midPoint, Color::BLACK, depthTest);
        debug->AddLine(midPoint, GetOtherWorldFrame().Translation(), Color::GRAY, depthTest);



        //draw the frames.
        const float axisLengths = 0.5f;

        float hueOffset = 0.05f;

        Color xAxisC;
        float shiftedColor = Color::RED.Hue() + hueOffset;
        if (shiftedColor > 1.0f)
            shiftedColor -= 1.0f;

        xAxisC.FromHSL(shiftedColor, Color::RED.SaturationHSL(), Color::RED.Lightness());

        Color yAxisC;
        shiftedColor = Color::GREEN.Hue() + hueOffset;
        if (shiftedColor > 1.0f)
            shiftedColor -= 1.0f;

        yAxisC.FromHSL(shiftedColor, Color::GREEN.SaturationHSL(), Color::GREEN.Lightness());

        Color zAxisC;
        shiftedColor = Color::BLUE.Hue() + hueOffset;
        if (shiftedColor > 1.0f)
            shiftedColor -= 1.0f;

        zAxisC.FromHSL(shiftedColor, Color::BLUE.SaturationHSL(), Color::BLUE.Lightness());



        Color xAxisDark = xAxisC.Lerp(Color::BLACK, 0.5f);
        Color yAxisDark = yAxisC.Lerp(Color::BLACK, 0.5f);
        Color zAxisDark = zAxisC.Lerp(Color::BLACK, 0.5f);

		//#todo re enable!
        //debug->AddFrame(GetOwnWorldFrame(), axisLengths, xAxisC, yAxisC, zAxisC, depthTest);
        //debug->AddFrame(GetOtherWorldFrame(), axisLengths, xAxisDark, yAxisDark, zAxisDark, depthTest);


        //draw the special joint stuff given to us by newton
        UrhoNewtonDebugDisplay debugDisplay(debug, depthTest);
        debugDisplay.SetDrawScale(1.0f);
        if (newtonJoint_)
        {
            newtonJoint_->Debug(&debugDisplay);//#todo this sometimes covers up the 2 frames above - maybe alter inside newton instead?
        }

    }

    void NewtonConstraint::MarkDirty(bool dirty /*= true*/)
    {
        dirty_ = dirty;
    }

    void NewtonConstraint::SetDisableCollision(bool disable)
    {
        enableBodyCollision_ = !disable;
        MarkDirty();
    }

    void NewtonConstraint::SetOtherBody(NewtonRigidBody* body)
    {

            if (otherBody_ != nullptr)
                RemoveJointReferenceFromBody(otherBody_);//remove reference from old body


            otherBody_ = body;

            AddJointReferenceToBody(body);
            body->GetNode()->AddListener(this);


            SetOtherWorldPosition(GetOwnWorldFrame().Translation());
            SetOtherWorldRotation(GetOwnWorldFrame().Rotation());


            otherBodyId_ = body->GetID();

            MarkDirty();
        
    }


    void NewtonConstraint::SetOtherBodyId(unsigned bodyId)
    {
        otherBodyId_ = bodyId;
        MarkDirty();
    }

    void NewtonConstraint::WakeBodies()
    {
        ownBody_->Activate();
        otherBody_->Activate(); 
    }

    void NewtonConstraint::SetWorldPosition(const Vector3& position)
    {
        SetOwnWorldPosition(position);
        SetOtherWorldPosition(position);
    }

    void NewtonConstraint::SetWorldRotation(const Quaternion& rotation)
    {
        SetOwnWorldRotation(rotation);
        SetOtherWorldRotation(rotation);
    }
    

    void NewtonConstraint::SetPosition(const Vector3& position)
    {
        SetOwnPosition(position);
        SetOtherWorldPosition(GetOwnWorldFrame().Translation());
    }

    void NewtonConstraint::SetRotation(const Quaternion& rotation)
    {
        SetOwnRotation(rotation);
        SetOtherWorldRotation(GetOwnWorldFrame().Rotation());
    }

    void NewtonConstraint::SetOwnPosition(const Vector3& position)
    {
        position_ = position;

        hasBeenBuilt_ = false;
        MarkDirty();
    }


    void NewtonConstraint::SetOwnRotation(const Quaternion& rotation)
    {
        rotation_ = rotation;
        hasBeenBuilt_ = false;
        MarkDirty();
    }


    void NewtonConstraint::SetOtherPosition(const Vector3& position)
    {
        otherPosition_ = position;
        hasBeenBuilt_ = false;
        MarkDirty();
    }


    void NewtonConstraint::SetOtherRotation(const Quaternion& rotation)
    {
        otherRotation_ = rotation;
        hasBeenBuilt_ = false;
        MarkDirty();
    }


    void NewtonConstraint::SetOwnWorldPosition(const Vector3& worldPosition)
    {
        SetOwnPosition(ownBody_->GetWorldTransform().Inverse() *  worldPosition);
    }

    void NewtonConstraint::SetOwnWorldRotation(const Quaternion& worldRotation)
    {
        SetOwnRotation(ownBody_->GetWorldRotation().Inverse() * worldRotation);
    } 

    void NewtonConstraint::SetOtherWorldPosition(const Vector3& position)
    {
        SetOtherPosition(otherBody_->GetWorldTransform().Inverse() * position);
    }

    void NewtonConstraint::SetOtherWorldRotation(const Quaternion& rotation)
    {
        SetOtherRotation(otherBody_->GetWorldRotation().Inverse() * rotation);
    }

    void NewtonConstraint::SetSolveMode(CONSTRAINT_SOLVE_MODE mode)
    {
        if (solveMode_ != mode) {
            solveMode_ = mode;
            applyAllJointParams();
        }
    }

    void NewtonConstraint::SetStiffness(float stiffness)
    {
        if (stiffness_ != stiffness) {
            stiffness_ = stiffness;
            applyAllJointParams();
        }
    }

    void NewtonConstraint::SetEnableForceCalculation(bool enabled)
    {
        if (enabled != enableForceCalculations_) {
            enableForceCalculations_ = enabled;
            applyAllJointParams();
        }
    }

    bool NewtonConstraint::GetEnableForceCalculation() const
    {
        return enableForceCalculations_;
    }

    Vector3 NewtonConstraint::GetOwnForce()
    {
        if(newtonJoint_&& enableForceCalculations_)
            return NewtonToUrhoVec3(newtonJoint_->GetForce0());

        return Vector3();
    }

    Vector3 NewtonConstraint::GetOtherForce()
    {
        if (newtonJoint_ && enableForceCalculations_)
            return NewtonToUrhoVec3(newtonJoint_->GetForce1());

        return Vector3();
    }

    Vector3 NewtonConstraint::GetOwnTorque()
    {
        if (newtonJoint_ && enableForceCalculations_)
            return NewtonToUrhoVec3(newtonJoint_->GetTorque0());

        return Vector3();
    }

    Vector3 NewtonConstraint::GetOtherTorque()
    {
        if (newtonJoint_ && enableForceCalculations_)
            return NewtonToUrhoVec3(newtonJoint_->GetTorque1());

        return Vector3();
    }

    NewtonBody* NewtonConstraint::GetOwnNewtonBody() const
    {
        return ownBody_->GetNewtonBody();
    }

    NewtonBody* NewtonConstraint::GetOtherNewtonBody() const
    {

        return otherBody_->GetNewtonBody();

    }

    void NewtonConstraint::BuildNow()
    {
        physicsWorld_->WaitForUpdateFinished();
        reEvalConstraint();
    }

    unsigned NewtonConstraint::GetOtherBodyId() const
    {
        return otherBodyId_;
    }

    Vector3 NewtonConstraint::GetOtherPosition() const
    {

       return otherPosition_;

    }

    Quaternion NewtonConstraint::GetOtherRotation() const
    {
        return otherRotation_;
    }

    Matrix3x4 NewtonConstraint::GetOwnWorldFrame() const
    {

        //return a frame with no scale at the position and rotation in node space
        Matrix3x4 worldFrame = ownBody_->GetWorldTransform() * Matrix3x4(position_, rotation_, 1.0f);


        //the frame could have uniform scale - reconstruct with no scale
        Matrix3x4 worldFrameNoScale = Matrix3x4(worldFrame.Translation(), worldFrame.Rotation(), 1.0f);


        return worldFrameNoScale;

    }

    Matrix3x4 NewtonConstraint::GetOtherWorldFrame() const
    {

        //return a frame with no scale at the position and rotation in node space.
        Matrix3x4 worldFrame = otherBody_->GetWorldTransform() * Matrix3x4(otherPosition_, otherRotation_, 1.0f);

        //the frame could have uniform scale - reconstruct with no scale
        Matrix3x4 worldFrameNoScale = Matrix3x4(worldFrame.Translation(), worldFrame.Rotation(), 1.0f);


        return worldFrameNoScale;
    }

    void NewtonConstraint::OnSetEnabled()
    {
        MarkDirty();
    }

    void NewtonConstraint::reEvalConstraint()
    {
        if (!IsEnabledEffective()) {
            freeInternal();
        }
        else if (ownBody_ && ownBody_->GetNewtonBody()) {
            freeInternal();

            bool goodToBuild = true;


            if (otherBodyId_ > 0) {
                NewtonRigidBody* body = (NewtonRigidBody*)GetScene()->GetComponent(otherBodyId_);
                if (body != otherBody_) {
                    if (body)
                        SetOtherBody(body);
                    else {
                        URHO3D_LOGWARNING("Contraint Could Not Resolve Other Body, Setting to Scene Body..");
                        SetOtherBody(physicsWorld_->sceneBody_);
                    }
                }
            }


            if (otherBody_->GetEffectiveMass() <= 0.0f && ownBody_->GetEffectiveMass() <= 0.0f) {
                goodToBuild = false;
                URHO3D_LOGWARNING("Contraint must connect to at least 1 Rigid Body with mass greater than 0.");
            }



            if (goodToBuild) {
                Matrix3x4 ownBodyLoadedTransform;
                Matrix3x4 otherBodyLoadedTransform;
                Vector3 ownBodyAngularVelocity;
                Vector3 otherBodyAngularVelocity;
                Vector3 ownBodyLinearVelocity;
                Vector3 otherBodyLinearVelocity;
                if (hasBeenBuilt_) {

                    //save loaded node state.
                    ownBodyLoadedTransform = ownBody_->GetWorldTransform();
                    ownBodyAngularVelocity = ownBody_->GetAngularVelocity();
                    ownBodyLinearVelocity = ownBody_->GetLinearVelocity();

                    otherBodyLoadedTransform = otherBody_->GetWorldTransform();
                    otherBodyAngularVelocity = otherBody_->GetAngularVelocity();
                    otherBodyLinearVelocity = otherBody_->GetLinearVelocity();

                    //set body to pre-Built Transform
                    otherBody_->SetWorldTransform(prevBuiltOtherBodyTransform_);
                    ownBody_->SetWorldTransform(prevBuiltOwnBodyTransform_);
                }

                buildConstraint();


                if (!hasBeenBuilt_) {
                    //save the state of bodies and pins after the first build.
                    prevBuiltOwnTransform_ = GetOwnNewtonBuildWorldFrame();
                    prevBuiltOwnBodyTransform_ = ownBody_->GetWorldTransform();

                    prevBuiltOtherTransform_ = GetOtherNewtonBuildWorldFrame();
                    prevBuiltOtherBodyTransform_ = otherBody_->GetWorldTransform();
                }
                else
                {
                    //restore node state
                    ownBody_->SetWorldTransform(ownBodyLoadedTransform);
                    ownBody_->SetLinearVelocity(ownBodyLinearVelocity, false);
                    ownBody_->SetAngularVelocity(ownBodyAngularVelocity);

                    otherBody_->SetWorldTransform(otherBodyLoadedTransform);
                    otherBody_->SetLinearVelocity(otherBodyLinearVelocity, false);
                    otherBody_->SetAngularVelocity(otherBodyAngularVelocity);

                }


                applyAllJointParams();


                hasBeenBuilt_ = true;
            }

        }
        else//we dont have own body so free the joint..
        {
            freeInternal();
        }






        MarkDirty(false);
    }

    void NewtonConstraint::buildConstraint()
    {
        /// ovverride in derived classes.
    }


    bool NewtonConstraint::applyAllJointParams()
    {
        WakeBodies();

        if (newtonJoint_ == nullptr)
            return false;

        /// extend in derived classes.
        NewtonJointSetCollisionState((NewtonJoint*)newtonJoint_, enableBodyCollision_);
        newtonJoint_->SetStiffness(stiffness_);
        newtonJoint_->SetJointForceCalculation(enableForceCalculations_);

        if(solveMode_ != SOLVE_MODE_JOINT_DEFAULT)
            newtonJoint_->SetSolverModel(solveMode_);

        return true;
    }

    void NewtonConstraint::freeInternal()
    {

        if (newtonJoint_ != nullptr) {
            physicsWorld_->addToFreeQueue(newtonJoint_);
            newtonJoint_ = nullptr;
        }
    }



    void NewtonConstraint::AddJointReferenceToBody(NewtonRigidBody* rigBody)
    {

        if (!rigBody->connectedConstraints_.contains(this))
            rigBody->connectedConstraints_.insert(this);

    }


    void NewtonConstraint::RemoveJointReferenceFromBody(NewtonRigidBody* rigBody)
    {

        if (rigBody->connectedConstraints_.contains(this))
            rigBody->connectedConstraints_.erase(this);

    }

    void NewtonConstraint::OnNodeSet(Node* node)
    {
        if (node)
        {
            //auto create physics world similar to rigid body.
            physicsWorld_ = node->GetScene()->GetOrCreateComponent<NewtonPhysicsWorld>();

            NewtonRigidBody* rigBody = node->GetComponent<NewtonRigidBody>();
            if (rigBody) {
                ownBody_ = rigBody;
                ownBodyId_ = ownBody_->GetID();
            }
           
            SetOtherBody(physicsWorld_->sceneBody_);

            physicsWorld_->addConstraint(this);

            AddJointReferenceToBody(ownBody_);

            node->AddListener(this);
        }
        else
        {
            if (!ownBody_.Expired())
                RemoveJointReferenceFromBody(ownBody_);

            if (!otherBody_.Expired())
                RemoveJointReferenceFromBody(otherBody_);


            ownBody_ = nullptr;
            if (!physicsWorld_.Expired())
                physicsWorld_->removeConstraint(this);

            freeInternal();

        }
    }

    void NewtonConstraint::OnNodeSetEnabled(Node* node)
    {
        MarkDirty();
    }



    Urho3D::Matrix3x4 NewtonConstraint::GetOwnBuildWorldFrame()
    {
        if (hasBeenBuilt_)
            return Matrix3x4(prevBuiltOwnTransform_.Translation(), prevBuiltOwnTransform_.Rotation(), 1.0f);
        else
            return GetOwnWorldFrame();
    }

    Urho3D::Matrix3x4 NewtonConstraint::GetOtherBuildWorldFrame()
    {
        if (hasBeenBuilt_)
            return Matrix3x4(prevBuiltOtherTransform_.Translation(), prevBuiltOtherTransform_.Rotation(), 1.0f);
        else
            return GetOtherWorldFrame();
    }

    Urho3D::Matrix3x4 NewtonConstraint::GetOwnNewtonBuildWorldFrame()
    {
        Matrix3x4 newtonWorldFrame = (GetOwnBuildWorldFrame());

        //newtonWorldFrame has scaling from the the physics world frame transformation. - reconstruct without scale because joints expect frame with no scaling.
        return Matrix3x4(newtonWorldFrame.Translation(), newtonWorldFrame.Rotation(), 1.0f);
    }

    Urho3D::Matrix3x4 NewtonConstraint::GetOtherNewtonBuildWorldFrame()
    {
        Matrix3x4 newtonWorldFrame = (GetOtherBuildWorldFrame());

        //newtonWorldFrame has scaling from the the physics world frame transformation. - reconstruct without scale because joints expect frame with no scaling.
        return Matrix3x4(newtonWorldFrame.Translation(), newtonWorldFrame.Rotation(), 1.0f);
    }



}
