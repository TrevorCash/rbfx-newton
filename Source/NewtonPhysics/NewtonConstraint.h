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

#pragma once
#include "UrhoNewtonApi.h"

#include "Urho3D/Scene/Component.h"

#include "ndNewton.h"


namespace Urho3D {
	class NewtonModel;

	class Context;
    class NewtonRigidBody;
    class NewtonPhysicsWorld;


    ///Base class for newton constraints.
    class URHONEWTON_API NewtonConstraint : public Component
    {
        URHO3D_OBJECT(NewtonConstraint, Component);


    public:

        friend class NewtonPhysicsWorld;
        friend class NewtonRigidBody;
        friend class NewtonModel;

        /// Construct.
        NewtonConstraint(Context* context);
        /// Destruct. Free the rigid body and geometries.
        ~NewtonConstraint() override;

        static void RegisterObject(Context* context);

        /// Visualize the component as debug geometry.
        virtual void DrawDebugGeometry(DebugRenderer* debug, bool depthTest) override;

        void MarkDirty(bool dirty = true);

		bool GetDirty() const { return dirty_; }

        /// Set whether to disable collisions between connected bodies.
        void SetDisableCollision(bool disable);

        /// Set other body to connect to. Set to null to connect to the static world.
        virtual void SetOtherBody(NewtonRigidBody* body, bool resetOtherPin = true);



        void SetOtherBodyId(unsigned bodyId);
        unsigned GetOtherBodyId() const;



        /// force wake the connected bodies
        void WakeBodies();

        ///set the world position of both frames on both bodies.
        void SetWorldPosition(const Vector3& position);
        ///set the world rotation of both frames on both bodies. 
        void SetWorldRotation(const Quaternion& rotation);

        /// set the position in local cordinates to own body and set the same world position on other body.
        void SetPosition(const Vector3& position);
        /// set the rotation in local cordinates to own body and set the same world rotation on other body.
        void SetRotation(const Quaternion& rotation);



        /// Set constraint position in local cordinates to rigidbody.
        void SetOwnPosition(const Vector3& position);
        /// set the rotational frame to use on own rigidbody 
        void SetOwnRotation(const Quaternion& rotation);

        Vector3 GetOwnPosition() const { return position_; }

        Quaternion GetOwnRotation() const { return rotation_; }

        /// Set constraint position in local cordinates relative to the other body. If connected to the static world, is a world space position.
        virtual void SetOtherPosition(const Vector3& position);
        /// set the rotational frame to use on other body. If connected to the static world, is a world space position.
        virtual void SetOtherRotation(const Quaternion& rotation);

       
        void SetOwnWorldPosition(const Vector3& worldPosition);

        void SetOwnWorldRotation(const Quaternion& worldRotation);


        /// Set constraint position in local cordinates relative to the other body. If connected to the static world, is a world space position.
        virtual void SetOtherWorldPosition(const Vector3& position);
        /// set the rotational frame to use on other body. If connected to the static world, is a world space position.
        virtual void SetOtherWorldRotation(const Quaternion& rotation);


        Vector3 GetOtherPosition() const;

        Quaternion GetOtherRotation() const;

        Matrix3x4 GetOwnWorldFrame() const;

        Matrix3x4 GetOtherWorldFrame() const;

        Vector3 GetOwnWorldFrameVel() const;
        Vector3 GetOwnWorldFrameOmega() const;
        Vector3 GetOtherWorldFrameVel() const;
        Vector3 GetOtherWorldFrameOmega() const;

        


        void SetSolveMode(ndJointBilateralSolverModel mode);
        void SetSolveMode(int mode) {
            SetSolveMode(ndJointBilateralSolverModel(mode));
        }

		ndJointBilateralSolverModel GetSolveMode() const { return solveMode_; }


        void SetStiffness(float stiffness);
        float GetStiffness() const { return stiffness_; }

        /// Enable force calculations on the joint.  Enable to use functions like GetOwnForce() etc..
        void SetEnableForceCalculation(bool enabled);

        bool GetEnableForceCalculation() const;

        ///return the force exerted on rigid body. Generally equal and opposite to GetOtherForce().  Only functional if ForceCalculation is enabled.
        Vector3 GetOwnForce();
        ///return the force exerted on other rigid body. Generally equal and opposite to GetOwnForce(). Only functional if ForceCalculation is enabled.
        Vector3 GetOtherForce();

        ///return the torque exerted on rigid body. Only functional if ForceCalculation is enabled.
        Vector3 GetOwnTorque();
        ///return the torque exerted on other rigid body. Only functional if ForceCalculation is enabled.
        Vector3 GetOtherTorque();

        //return the relative angular vel between the 2 bodies in world space
        Vector3 GetWorldAngularRate();


        /// Return physics world.
        NewtonPhysicsWorld* GetPhysicsWorld() const { return physicsWorld_; }

        /// Return rigid body in own scene node.
        NewtonRigidBody* GetOwnBody(bool resolved = true) const;

		ndBody* GetOwnNewtonBody(bool resolved = true) const;
		ndBody* GetOwnNewtonBodyBuild() const;

        /// Return the other rigid body. May be null if connected to the static world.
        NewtonRigidBody* GetOtherBody(bool resolved = true) const;

		ndBody* GetOtherNewtonBody(bool resolved = true) const;
		ndBody* GetOtherNewtonBodyBuild() const;

        NewtonModel* GetModel() const;

        ndConstraint* GetNewtonJoint() const {
            return  newtonConstraint_;
        }

        virtual void OnSetEnabled() override;


		virtual void OnSetAttribute(const AttributeInfo& attr, const Variant& src) override;



	protected:
        /// Physics world.
        WeakPtr<NewtonPhysicsWorld> physicsWorld_;
        /// Own rigid body.
		WeakPtr<NewtonRigidBody> ownBody_;
        unsigned ownBodyId_ = 0;

        /// Other rigid body.
		WeakPtr<NewtonRigidBody> otherBody_;
        unsigned otherBodyId_ = 0;

		WeakPtr<NewtonRigidBody> ownBodyResolved_;
		WeakPtr<NewtonRigidBody> otherBodyResolved_;


        /// the newton model collection that the constraint is part of.
        WeakPtr<NewtonModel> model_;

        /// Internal newtonJoint.
        ndConstraint* newtonConstraint_ = nullptr;
        /// Flag indicating the two bodies should collide with each other.
        bool enableBodyCollision_ = false;


        /// Constraint position local to other body
        Vector3 otherPosition_;
        Quaternion otherRotation_;
        /// Constraint position local to body
        Vector3 position_;
        Quaternion rotation_;


        float stiffness_ = 0.7f;

		ndJointBilateralSolverModel solveMode_ = m_jointkinematicOpenLoop;

        bool graphTraverseFlag = false;




        Matrix3x4 initialBuiltOwnBodyTransform_;
        Matrix3x4 initialBuiltOtherBodyTransform_;
        Matrix3x4 initialBuiltOwnWorldPinTransform_;
        Matrix3x4 initialBuiltOtherWorldPinTransform_;
		NewtonRigidBody* prevBuiltOwnBody_;
		NewtonRigidBody* prevBuiltOtherBody_;
        bool hasBeenBuilt_ = false;




        ///dirty flag.
        bool dirty_ = true;

        bool enableForceCalculations_ = false;

        /// Upper level re-evaulation.
        void reEvalConstraint();
        
		NewtonRigidBody* resolveBody(NewtonRigidBody* body);
		NewtonRigidBody* resolveBody(Node* body);

        /// build the newton constraint.
        virtual void buildConstraint();

        /// update params on the already build constraint
        virtual bool applyAllJointParams();
        
        /// frees and deletes the internal joint.
        void freeInternal();

        void AddJointReferenceToBody(NewtonRigidBody* rigBody);

        void RemoveJointReferenceFromBody(NewtonRigidBody* rigBody);



        virtual void OnNodeSet(Node* node) override;
        virtual void OnNodeSetEnabled(Node* node) override;

        Matrix3x4 GetOwnBuildWorldFrame();
        Matrix3x4 GetOtherBuildWorldFrame();
    };
}

