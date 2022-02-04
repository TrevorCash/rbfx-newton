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
#include "NewtonModel.h"


#include "Urho3D/Core/Context.h"
#include "Urho3D/Scene/Component.h"
#include "Urho3D/Graphics/DebugRenderer.h"
#include "Urho3D/Scene/Scene.h"
#include "Urho3D/IO/Log.h"




namespace Urho3D {


    const char* solveModeNames[] =
    {
        "m_jointIterativeSoft",
        "m_jointkinematicOpenLoop",
        "m_jointkinematicCloseLoop",
        "m_jointkinematicAttachment",
        nullptr
    };


    NewtonConstraint::NewtonConstraint(Context* context) : Component(context)
    {

    }

    NewtonConstraint::~NewtonConstraint()
    {
		//URHO3D_LOGINFO("NewtonConstraint Destructor");
    }

    void NewtonConstraint::RegisterObject(Context* context)
    {
        context->RegisterFactory<NewtonConstraint>(DEF_PHYSICS_CATEGORY.c_str());
        URHO3D_COPY_BASE_ATTRIBUTES(Component);

        URHO3D_ENUM_ACCESSOR_ATTRIBUTE("Solver Iterations", GetSolveMode, SetSolveMode, ndJointBilateralSolverModel, solveModeNames, m_jointkinematicOpenLoop, AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("Stiffness", GetStiffness, SetStiffness, float, 0.7f, AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("ForceCalculationsEnabled", GetEnableForceCalculation, SetEnableForceCalculation, bool, false, AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("Other Body ID", GetOtherBodyId, SetOtherBodyId, unsigned, 0, AM_DEFAULT | AM_COMPONENTID);

        URHO3D_ATTRIBUTE("Prev Built Own Transform", Matrix3x4, initialBuiltOwnWorldPinTransform_, Matrix3x4::IDENTITY, AM_DEFAULT);
        URHO3D_ATTRIBUTE("Prev Built Other Transform", Matrix3x4, initialBuiltOtherWorldPinTransform_, Matrix3x4::IDENTITY, AM_DEFAULT);
        URHO3D_ATTRIBUTE("Prev Built Own Body Transform", Matrix3x4, initialBuiltOwnBodyTransform_, Matrix3x4::IDENTITY, AM_DEFAULT);
        URHO3D_ATTRIBUTE("Prev Built Other Body Transform", Matrix3x4, initialBuiltOtherBodyTransform_, Matrix3x4::IDENTITY, AM_DEFAULT);
        URHO3D_ATTRIBUTE("Has Been Built", bool, hasBeenBuilt_, false, AM_DEFAULT);


        URHO3D_ATTRIBUTE("Other Body Frame Position", Vector3, otherPosition_, Vector3::ZERO, AM_DEFAULT);
        URHO3D_ATTRIBUTE("Other Body Frame Rotation", Quaternion, otherRotation_, Quaternion::IDENTITY, AM_DEFAULT);
        URHO3D_ATTRIBUTE("Body Frame Position", Vector3, position_, Vector3::ZERO, AM_DEFAULT);
        URHO3D_ATTRIBUTE("Body Frame Rotation", Quaternion, rotation_, Quaternion::IDENTITY, AM_DEFAULT);
    }

    void NewtonConstraint::DrawDebugGeometry(DebugRenderer* debug, bool depthTest)
    {
        if (!IsEnabled())
            return;


		float scale = physicsWorld_->debugScale_;

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

		
        debug->AddFrame(GetOwnWorldFrame(), axisLengths*scale, xAxisC, yAxisC, zAxisC, depthTest);
        debug->AddFrame(GetOtherWorldFrame(), axisLengths*scale, xAxisDark, yAxisDark, zAxisDark, depthTest);


        //draw the special joint stuff given by newton
        NewtonConstraintDebugCallback debugDisplay(debug, depthTest);
        debugDisplay.SetDrawScale(1.0f*scale);
        if (newtonConstraint_)
        {
			newtonConstraint_->GetAsBilateral()->DebugJoint(static_cast<ndConstraintDebugCallback&>(debugDisplay));//#todo this sometimes covers up the 2 frames above - maybe alter inside newton instead?
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

    void NewtonConstraint::SetOtherBody(NewtonRigidBody* body, bool resetOtherPin /*= true*/)
    {

            if (otherBody_ != nullptr)
                RemoveJointReferenceFromBody(otherBody_);//remove reference from old body

			otherBody_ = body;
            otherBodyResolved_ = resolveBody(body);

            AddJointReferenceToBody(body);
            body->GetNode()->AddListener(this);

			if (resetOtherPin) {
				SetOtherWorldPosition(GetOwnWorldFrame().Translation());
				SetOtherWorldRotation(GetOwnWorldFrame().Rotation());
			}

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
		if(!ownBody_.Expired())
			ownBody_->Activate();

		if(!otherBody_.Expired())
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

    void NewtonConstraint::SetSolveMode(ndJointBilateralSolverModel mode)
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
        if(newtonConstraint_&& enableForceCalculations_)
            return NewtonToUrhoVec3(newtonConstraint_->GetAsBilateral()->GetForceBody0());

        return Vector3();
    }

    Vector3 NewtonConstraint::GetOtherForce()
    {
        if (newtonConstraint_ && enableForceCalculations_)
            return NewtonToUrhoVec3(newtonConstraint_->GetAsBilateral()->GetForceBody1());

        return Vector3();
    }

    Vector3 NewtonConstraint::GetOwnTorque()
    {
        if (newtonConstraint_ && enableForceCalculations_)
            return NewtonToUrhoVec3(newtonConstraint_->GetAsBilateral()->GetTorqueBody0());

        return Vector3();
    }

    Vector3 NewtonConstraint::GetOtherTorque()
    {
        if (newtonConstraint_ && enableForceCalculations_)
            return NewtonToUrhoVec3(newtonConstraint_->GetAsBilateral()->GetTorqueBody1());

        return Vector3();
    }

    Vector3 NewtonConstraint::GetWorldAngularRate()
    {
        if(newtonConstraint_)
        {
            ndVector relOmega = newtonConstraint_->GetBody0()->GetOmega() - newtonConstraint_->GetBody1()->GetOmega();
            return NewtonToUrhoVec3(relOmega);
        }
        return Vector3::ZERO;
    }


    Urho3D::NewtonRigidBody* NewtonConstraint::GetOwnBody(bool useResolved /*= true*/) const
	{
		if (useResolved && (ownBodyResolved_ != ownBody_))
		{
			return ownBodyResolved_;
		}
		else
			return ownBody_;
	}

	ndBody* NewtonConstraint::GetOwnNewtonBody(bool useResolved /*= true */) const
    {
        return GetOwnBody(useResolved)->GetNewtonBody();
    }

	ndBody* NewtonConstraint::GetOwnNewtonBodyBuild() const
	{

		return GetOwnNewtonBody(true);
	}

	Urho3D::NewtonRigidBody* NewtonConstraint::GetOtherBody(bool useResolved /*= true*/) const
	{
		if (useResolved && (otherBodyResolved_ != otherBody_))
			return otherBodyResolved_;
		else
			return otherBody_;
	}

	ndBody* NewtonConstraint::GetOtherNewtonBody(bool resolved /*= true*/) const
    {
        return GetOtherBody(resolved)->GetNewtonBody();
    }

	ndBody* NewtonConstraint::GetOtherNewtonBodyBuild() const
	{
		return GetOtherNewtonBody(true);
	}

	NewtonModel* NewtonConstraint::GetModel() const
	{
        return model_;
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
        return worldFrame;
    }

    Matrix3x4 NewtonConstraint::GetOtherWorldFrame() const
    {

        //return a frame with no scale at the position and rotation in node space.
        Matrix3x4 worldFrame = otherBody_->GetWorldTransform() * Matrix3x4(otherPosition_, otherRotation_, 1.0f);
        return worldFrame;
    }

    Vector3 NewtonConstraint::GetOwnWorldFrameVel() const
    {
        Vector3 worldDelta = GetOwnWorldFrame().Translation() - ownBody_->GetWorldTransform().Translation();
        return ownBody_->GetLinearVelocity(TS_WORLD) + ownBody_->GetAngularVelocity(TS_WORLD).CrossProduct(worldDelta);
    }

    Vector3 NewtonConstraint::GetOwnWorldFrameOmega() const
    {
        return ownBody_->GetAngularVelocity(TS_WORLD);
    }

    Vector3 NewtonConstraint::GetOtherWorldFrameVel() const
    {
        Vector3 worldDelta = GetOtherWorldFrame().Translation() - otherBody_->GetWorldTransform().Translation();
        return otherBody_->GetLinearVelocity(TS_WORLD) + otherBody_->GetAngularVelocity(TS_WORLD).CrossProduct(worldDelta);
    }

    Vector3 NewtonConstraint::GetOtherWorldFrameOmega() const
    {
        return ownBody_->GetAngularVelocity(TS_WORLD);
    }

    void NewtonConstraint::OnSetEnabled()
    {
        MarkDirty();
    }

	void NewtonConstraint::OnSetAttribute(const AttributeInfo& attr, const Variant& src)
	{
		Component::OnSetAttribute(attr, src);
	}


	void NewtonConstraint::reEvalConstraint()
	{
		NewtonRigidBody* ownBodyResolvedPrev = ownBodyResolved_;
		NewtonRigidBody* otherBodyResolvedPrev = otherBodyResolved_;

		ownBodyResolved_ = resolveBody(ownBody_);

        if (!IsEnabledEffective()) {
            freeInternal();
        }
        else if (ownBodyResolved_ && ownBodyResolved_->GetNewtonBody())
		{
 
			freeInternal();

            if (otherBodyId_ > 0) {
                NewtonRigidBody* body = (NewtonRigidBody*)GetScene()->GetComponent(otherBodyId_);
                if (body != otherBody_) {
					if (body) {
						
						bool curHasBeenBuilt = hasBeenBuilt_;
						SetOtherBody(body, false);
						hasBeenBuilt_ = curHasBeenBuilt;//preserve hasBeenBuilt flag in this case!

					}
                    else {
                        URHO3D_LOGWARNING("Contraint Could Not Resolve Other Body, Setting to Scene Body..");
                        SetOtherBody(physicsWorld_->sceneBody_);
                    }
                }
            }


			otherBodyResolved_ = resolveBody(otherBody_);



            Matrix3x4 ownBodyLoadedTransform;
            Matrix3x4 otherBodyLoadedTransform;
            Vector3 ownBodyAngularVelocity;
            Vector3 otherBodyAngularVelocity;
            Vector3 ownBodyLinearVelocity;
            Vector3 otherBodyLinearVelocity;
            
			
			if (hasBeenBuilt_) {


				//save current node state. (normal case)
				ownBodyLoadedTransform = ownBodyResolved_->GetWorldTransform();
				ownBodyAngularVelocity = ownBodyResolved_->GetAngularVelocity();
				ownBodyLinearVelocity = ownBodyResolved_->GetLinearVelocity();



				otherBodyLoadedTransform = otherBodyResolved_->GetWorldTransform();
				otherBodyAngularVelocity = otherBodyResolved_->GetAngularVelocity();
				otherBodyLinearVelocity = otherBodyResolved_->GetLinearVelocity();

				
				//set body to pre-Built Transform
				if (ownBody_ != ownBodyResolved_)
				{
					//we have just switched to a new body - we need to adjust the resolved body to a position where the ownBody_ is at initialBuiltOwnBodyTransform_.
					Matrix3x4 localSpaceToOld = initialBuiltOwnBodyTransform_.Inverse() * ownBodyResolved_->GetWorldTransform();
					ownBodyResolved_->SetWorldTransform(initialBuiltOwnBodyTransform_ * localSpaceToOld );
				}
				else
				{
					ownBodyResolved_->SetWorldTransform(initialBuiltOwnBodyTransform_);
				}

				if (otherBody_ != otherBodyResolved_)
				{
					//we have just switched to a new body - we need to adjust the resolved body to a position where the otherBody_ is at initialBuiltOtherBodyTransform_.
					Matrix3x4 localSpaceToOld = initialBuiltOtherBodyTransform_.Inverse() * otherBodyResolved_->GetWorldTransform();
					otherBodyResolved_->SetWorldTransform(initialBuiltOtherBodyTransform_ * localSpaceToOld);
				}
				else
				{
					otherBodyResolved_->SetWorldTransform(initialBuiltOtherBodyTransform_);
				}
					
            }

			//URHO3D_LOGINFO("Attempting to build constraint..");
			//its possible that the resolved bodies could be the same body, if so, continue without actually building.
			if (ownBodyResolved_ != otherBodyResolved_) {
				
				//make sure neither body is kinematic
				if (!ownBodyResolved_->isKinematic_ && !otherBodyResolved_->isKinematic_)
				{
					//make sure at least one body has mass.
					if (!(ownBodyResolved_->GetEffectiveMass() <= 0.0f && otherBodyResolved_->GetEffectiveMass() <= 0.0f)) {
						//URHO3D_LOGINFO("building constraint " + ea::to_string((int)(void*)this));

						buildConstraint();
						if(newtonConstraint_ != nullptr)
							physicsWorld_->GetNewtonWorld()->AddJoint(newtonConstraint_->GetAsBilateral());
					}
				}
				
			}


            if (!hasBeenBuilt_) {
				
                //save the state of bodies and pins after the first build
                initialBuiltOwnBodyTransform_ = ownBodyResolved_->GetWorldTransform();
                initialBuiltOtherBodyTransform_ = otherBodyResolved_->GetWorldTransform();
				//URHO3D_LOGINFO("saving initial transforms.");
            }
            else
            {
                //restore body states
				ownBodyResolved_->SetWorldTransform(ownBodyLoadedTransform);
				ownBodyResolved_->SetLinearVelocity(ownBodyLinearVelocity, false);
				ownBodyResolved_->SetAngularVelocity(ownBodyAngularVelocity);

				otherBodyResolved_->SetWorldTransform(otherBodyLoadedTransform);
				otherBodyResolved_->SetLinearVelocity(otherBodyLinearVelocity, false);
				otherBodyResolved_->SetAngularVelocity(otherBodyAngularVelocity);
            }


            applyAllJointParams();


			

            hasBeenBuilt_ = true;
        }
        else//we don't have own body so free the joint..
        {
            freeInternal();
        }






        MarkDirty(false);
    }



	Urho3D::NewtonRigidBody* NewtonConstraint::resolveBody(NewtonRigidBody* body)
	{
		ea::vector<NewtonRigidBody*> rigidBodies;
		GetRootRigidBodies(rigidBodies, body->GetNode(), true);

		for (NewtonRigidBody* rb : rigidBodies)
		{
			if (rb->IsEnabled()) {
				//if(rb != body)
					//URHO3D_LOGINFO("NewtonConstraint(" + ea::to_string((long)(void*)this) + "): body resolved from node name " + body->GetNode()->GetName() + " to node name " + rb->GetNode()->GetName());
				
				return rb;
			}
		}

		return body;
	}

	Urho3D::NewtonRigidBody* NewtonConstraint::resolveBody(Node* node)
	{
		ea::vector<NewtonRigidBody*> rigidBodies;
		GetRootRigidBodies(rigidBodies, node, true);

		for (NewtonRigidBody* rb : rigidBodies)
		{
			if (rb->IsEnabled()) {
				//if(rb != body)
					//URHO3D_LOGINFO("NewtonConstraint(" + ea::to_string((long)(void*)this) + "): body resolved from node name " + body->GetNode()->GetName() + " to node name " + rb->GetNode()->GetName());

				return rb;
			}
		}

		return nullptr;
	}

    void NewtonConstraint::buildConstraint()
    {
        /// ovverride in derived classes.
    }


    bool NewtonConstraint::applyAllJointParams()
    {
        WakeBodies();

        if (newtonConstraint_ == nullptr)
            return false;

        /// extend in derived classes.
		newtonConstraint_->GetAsBilateral()->SetCollidable(enableBodyCollision_);

        if(solveMode_ != m_jointkinematicOpenLoop)
            newtonConstraint_->GetAsBilateral()->SetSolverModel(solveMode_);

        return true;
    }

    void NewtonConstraint::freeInternal()
    {
        if (newtonConstraint_ != nullptr) {
			
            physicsWorld_->freeConstraintQueue_.push_back(newtonConstraint_);
            newtonConstraint_ = nullptr;
        }
    }



    void NewtonConstraint::AddJointReferenceToBody(NewtonRigidBody* rigBody)
    {

		if (!rigBody->connectedConstraints_.contains(WeakPtr<NewtonConstraint>(this)))
		{
			ea::vector<NewtonRigidBody*> connectedBodies;
			rigBody->GetConnectedBodies(connectedBodies);

			NewtonRigidBody* pairedBody = nullptr;
			if (GetOwnBody() == rigBody)
				pairedBody = GetOtherBody();
			if (GetOtherBody() == rigBody)
				pairedBody = GetOwnBody();

			//see if rigBody already is connected to the pairedBody.  if so raise warning/error
			if(connectedBodies.contains(pairedBody))
			{
				URHO3D_LOGERROR("MULTIPLE CONTRAINTS CONNECTING 2 BODIES");
				URHO3D_ASSERT(0);
			}

			rigBody->connectedConstraints_.insert(WeakPtr<NewtonConstraint>(this));
		}



    }


    void NewtonConstraint::RemoveJointReferenceFromBody(NewtonRigidBody* rigBody)
    {

        if (rigBody->connectedConstraints_.contains(WeakPtr<NewtonConstraint>(this)))
            rigBody->connectedConstraints_.erase(WeakPtr<NewtonConstraint>(this));

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

			//resolve ownBody just in case it should actually belong to a parent body.
			ownBodyResolved_ = resolveBody(ownBody_);

            
            SetOtherBody(physicsWorld_->sceneBody_, false);

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
            return GetOwnWorldFrame();
    }

    Urho3D::Matrix3x4 NewtonConstraint::GetOtherBuildWorldFrame()
    {
            return GetOtherWorldFrame();
    }

}
