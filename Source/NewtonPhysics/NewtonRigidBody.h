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
#include "Urho3D/Scene/Node.h"

#include "ndNewton.h"


#include "EASTL/hash_set.h"


class NewtonBody;

namespace Urho3D
{

    class NewtonConstraint;
    class Component;
    class NewtonPhysicsWorld;
    class NewtonCollisionShape;
    class NewtonRigidBodyContactEntry;
	class NewtonBodyNotifications;

    static const char* RigidBodyCollisionEventModeNames[] =
    {
        "COLLISION_NEVER",
        "COLLISION_ALL",
        "COLLISION_START_END",
        nullptr
    };

    static const unsigned DEFAULT_COLLISION_LAYER = 0;
    static const unsigned DEFAULT_COLLISION_MASK = M_MAX_UNSIGNED;//default collide with all layers.


    struct RigidBodyCollisionExceptionEntry
    {
        unsigned rigidBodyComponentId_ = M_MAX_UNSIGNED;
        bool enableCollisions_ = false;
    };




    class URHONEWTON_API NewtonRigidBody : public Component
    {
        URHO3D_OBJECT(NewtonRigidBody, Component);
    public:

        /// Rigid body collision event signaling mode.
        enum RigidBodyCollisionEventMode
        {
            COLLISION_NEVER = 0,    //dont generate any collision events
            COLLISION_ALL = 1,          //generate all collision events start, during, and end.
            COLLISION_START_END = 2,    //only generate start of collision and end of collision.
        };



        friend class NewtonCollisionShape;
        friend class NewtonConstraint;
        friend class NewtonPhysicsWorld;
        friend class NewtonModel;

        /// Construct.
        NewtonRigidBody(Context* context);
        /// Destruct. Free the rigid body and geometries.
        ~NewtonRigidBody() override;
        /// Register object factory.
        static void RegisterObject(Context* context);

        ///Set a scaler on the mass of the rigid body - (scalar is applied to collision shape densities)
        void SetMassScale(float massDensityScale);



        NewtonPhysicsWorld* GetPhysicsWorld() const { return physicsWorld_; }

        ///  Set the world transform of the body's reference frame to the transform of the node.
        void SetWorldTransformToNode();

        ///  Set the world transform of the body in scene space
        void SetWorldTransform(const Matrix3x4& transform);
        void SetWorldPosition(const Vector3& position);
        void SetWorldRotation(const Quaternion& quaternion);

        
        /// returns the body transform matrix
        Matrix3x4 GetWorldTransform();
        Vector3 GetWorldPosition();
        Quaternion GetWorldRotation();

		/// center of mass
		void SetCenterOfMassLocalOffset(const Vector3& offset);
		Vector3 GetCenterOfMassLocalOffset() const { return centerOfMassEffective_; }
		Matrix3x4 GetCOMWorldTransform();
		void ResetCenterOfMass();


        ///Get the mass scale of the rigid body
        float GetMassScale() const { return massScale_; }

        /// get the mass of the rigid body
        float GetEffectiveMass() const { return mass_; }

        /// set the collision layer
        void SetCollisionLayer(unsigned layer);
        unsigned GetCollisionLayer() const { return collisionLayer_; }

        /// set the collision mask that specifies what other layers you can collide with
        void SetCollisionLayerMask(unsigned mask);
        void SetCollidableLayer(unsigned layer);
        void UnSetCollidableLayer(unsigned layer);

        unsigned GetCollisionLayerMask() const { return collisionLayerMask_; }


        /// Set a collision exception with another body.  This will override any collision layers/masks.
        void SetCollisionOverride(NewtonRigidBody* otherBody, bool enableCollisions);
        void SetCollisionOverride(unsigned otherBodyId, bool enableCollisions);
        void RemoveCollisionOverride(NewtonRigidBody* otherBody);
        void RemoveCollisionOverride(unsigned otherBodyId);
        void GetCollisionExceptions(ea::vector<RigidBodyCollisionExceptionEntry>& exceptions);
        void ClearCollisionExceptions() { collisionExceptions_.clear(); }

        /// make this body not collide with anything.
        void SetNoCollideOverride(bool noCollide);
        bool GetNoCollideOverride() const { return noCollideOverride_; }

        ///returns true if this body can collide with the specified body given current collision layers/masks and exceptions.
        bool CanCollideWith(NewtonRigidBody* otherBody);


        ///set collision event mode
        void SetCollisionEventMode(RigidBodyCollisionEventMode mode) {
            if (collisionEventMode_ != mode) {
                collisionEventMode_ = mode;
            }
        }

        

        ///trigger mode will not collide with anything but will still generate contacts and send collision events.
        void SetTriggerMode(bool enable) {
            if (triggerMode_ != enable) {
                triggerMode_ = enable;
            }
        }
        bool GetTriggerMode() const { return triggerMode_; }


        void SetIsKinematic(bool enable) {
            if (isKinematic_ != enable)
            {
                isKinematic_ = enable;
                MarkDirty();
            }
           
        }
        bool GetIsKinematic() const { return isKinematic_; }

        //set Ix, Iy, and Iz to all be equal to max(Ix, Iy, Iz).
        void SetUseInertiaHack(bool useInertiaHack)
        {
            useInertiaHack_ = useInertiaHack;
        }
        bool GetUseInertiaHack() const { return useInertiaHack_; }


        void SetGenerateContacts(bool enable)
        {
            generateContacts_ = enable;
        }
        bool GetGenerateContacts() const { return generateContacts_; }


		/// 
		Vector3 GetLinearMomentum() const { return GetLinearVelocity() * GetEffectiveMass(); }

		Vector3 GetAngularMomentum()  const;

        /// Set linear velocity in world coordinates. if useForces is false the velocity will be set exactly with no regard to using forces to achieve the desired velocity.
        void SetLinearVelocity(const Vector3& worldVelocity, bool useForces = true);

        void SetLinearVelocityHard(const Vector3& worldVelocity);


        /// Set the Angular velocity in world cordinates (degrees/sec) about COM
        void SetAngularVelocity(const Vector3& angularVelocity);


        /// Set linear damping factor (0.0 to 1.0) default is 0
        void SetLinearDamping(float dampingFactor);

        float GetLinearDamping() const { return linearDampening_; }

        /// Set Angular Damping factor for angle component. default is 0
        void SetAngularDamping(float angularDamping);

        float GetAngularDamping() const { return angularDampening_; }

        /// Set the internal linear damping - this is used internally by the newton solver to bring bodies to sleep more effectively.
        /// Be Aware that this parameter will show different behalvior with different solver iteration rates. (0.0 to 1.0) default is zero
        void SetInternalLinearDamping(float damping);

        /// Set the internal angular damping - this is used internally by the newton solver to bring bodies to sleep more effectively.
        /// Be Aware that this parameter will show different behalvior with different solver iteration rates. (0.0 to 1.0) default is zero
        void SetInternalAngularDamping(float angularDamping);

        /// Set the interpolation duration for applying transform to the scene node. 1.0f is no interpolation, 0.0f is inifinitely slow interpolation.
       // void SetInterpolationFactor(float factor = 0.0f);

        //float GetInterpolationFactor() const { return interpolationFactor_; }

        /// returns true if the interpolation is within tolerance of target rigidbody value.
        //bool InterpolationWithinRestTolerance();

        /// snap current interpolated values directly to target values.
        //void SnapInterpolation();

		/// look at all rigid body children and use their momentums.
		void ApplyMomentumFromRigidBodyChildren(bool clearChildrenVelocities);


        void SetAutoSleep(bool enableAutoSleep);

        bool GetAutoSleep() const { return autoSleep_; }

        bool GetAwake() const;

        /// force the body to be awake
        void Activate();
        /// force the body to sleep
        void DeActivate();


        /// Add a force to the body in world cordinates on the body's center of mass.
        void AddWorldForce(const Vector3& force);
        /// Add a force to the body in world cordinates at worldPosition.
        void AddWorldForce(const Vector3& force, const Vector3& worldPosition);
        /// Add a torque to the body in world space
        void AddWorldTorque(const Vector3& torque);

        /// Add a force to the body in local cordinates on the body's center of mass.
        void AddLocalForce(const Vector3& force);
        /// Add a force to the body in local cordinates localPosition from the body's center of mass.
        void AddLocalForce(const Vector3& force, const Vector3& localPosition);
        /// Add a torque to the body in local space
        void AddLocalTorque(const Vector3& torque);

        /// Reset accumulated forces.
        void ResetForces();

        /// apply an impulse to the body at the localPosition to aquire the target velocity next physics update.
        void AddImpulse(const Vector3& localPosition, const Vector3& targetWorldVelocity);




        /// Return the net force acting on the body.
        Vector3 GetNetWorldForce();

		/// sets the net world force on the body
		void SetNetWorldForce(Vector3 force);

        /// Return the net torque acting on the body.
        Vector3 GetNetWorldTorque();

		/// sets the net world torque on the body.
		void SetNetWorldTorque(Vector3 torque);

        ///Get the currently used newton body.
        ndBody* GetNewtonBody() const { return newtonBody_; }
        /// Return the currently used newton collision
        ndShapeInstance& GetEffectiveNewtonShape() const;


        NewtonModel* GetModel() const { return model_; }

        Vector3 GetLinearVelocity(TransformSpace space = TS_WORLD) const;


        Vector3 GetAngularVelocity(TransformSpace space = TS_WORLD) const;


        Vector3 GetAcceleration();

		Matrix3 GetMassMatrix();


        /// Get Immediately connected contraints to this rigid body.
        void GetConnectedContraints(ea::vector<NewtonConstraint*>& contraints);
		void GetConnectedBodies(ea::vector<NewtonRigidBody*>& bodies);
		ea::vector<NewtonConstraint*> GetConnectedContraints();

		ea::vector<NewtonCollisionShape*> GetCollisionShapes() const { return collisionShapes_; }

        ///Apply the current newton body transform to the node.
        void ApplyTransformToNode();

        ///Return the net force and torque for newton.
        void GetForceAndTorque(Vector3& force, Vector3& torque);

        int GetSceneDepth() { return sceneDepth_; }

        /// Draw Debug geometry
        void DrawDebugGeometry(DebugRenderer* debug, 
			bool depthTest, 
			bool showAABB = true, 
			bool showCollisionMesh = true, 
			bool showCenterOfMass = true,
			bool showContactForces = true, 
			bool showBodyFrame = true);

    	void DrawCollision(DebugRenderer* debug, bool depthTest /*= false*/);

        /// mark the rigid body as dirty causing the newton rigid body to be rebuilt by the physics world
        void MarkDirty(bool dirty = true);

        bool GetDirty() const { return needsRebuilt_; }


        /// mark the internal newton transform as dirty indicating the transform needs to be copied to the node.
        void MarkInternalTransformDirty(bool dirty = true);

        bool GetInternalTransformDirty();



        virtual void OnSetEnabled() override;




	protected:


        /// Internal newton body
        ndBody * newtonBody_ = nullptr;

		/// notifications objects
		NewtonBodyNotifications* newtonNotifications_ = nullptr;

        /// compound collision if needed.
        ndShapeInstance effectiveCollision_ = nullptr;
        /// Physics world.
		WeakPtr<NewtonPhysicsWorld> physicsWorld_;
        /// all currently used collision shape components.
		ea::vector<NewtonCollisionShape*> collisionShapes_;

        /// the newton model collection that the body is part of.
        WeakPtr<NewtonModel> model_;

        bool sceneRootBodyMode_ = false;
        ///Continuous Collision
        bool continuousCollision_ = false;

        ///Gyro Forces
        bool enableGyroTorque_ = false;

        /// flag indicating debug geometry for the collision should be shown in the debug renderer
        bool drawPhysicsDebugCollisionGeometry_ = false;


        RigidBodyCollisionEventMode collisionEventMode_ = COLLISION_START_END;


        Node* prevNode_ = nullptr;

        ///Net Force in local cordinates
        Vector3 netForce_;
        ///Net Torque in local cordinates
        Vector3 netTorque_;
        ///angular dampending
        float angularDampening_ = 0.0f;
        ///linera dampening
        float linearDampening_ = 0.0f;
        ///angular dampending
        Vector3 angularDampeningInternal_;
        ///linera dampening
        float linearDampeningInternal_ = 0.0f;

        ///currently connected constraints.
        ea::hash_set<WeakPtr<NewtonConstraint>> connectedConstraints_;

		///the current center of mass
		Vector3 centerOfMassEffective_;
		Vector3 centerOfMassCalculated_;

		Vector3 localCOMOffsetOverride_;
		bool useCOMOffsetOverride_ = false;


        ndVector netForceNewton_;
        ndVector netTorqueNewton_;

        ///effective mass
        float mass_ = 0.0f;
        ///mass scale
        float massScale_ = 1.0f;

        bool autoSleep_ = true;

        unsigned collisionLayer_ = DEFAULT_COLLISION_LAYER;
        unsigned collisionLayerMask_ = DEFAULT_COLLISION_MASK;

        //HashMap<unsigned, RigidBodyCollisionExceptionEntry> collisionExceptions_;
        VariantMap collisionExceptions_;

        bool noCollideOverride_ = false;

        bool triggerMode_ = false;

        bool generateContacts_ = true;

        bool isKinematic_ = false;

        bool useInertiaHack_ = false;

        ///dirty flag
        bool needsRebuilt_ = true;

        /// flag indicating the newton body has changed transforms and needs to update the node.
        bool transformDirty_ = true;

        /// last set node transform in ApplyTransform.
        Matrix3x4 lastSetNodeWorldTransform_;

        /// how many node levels deep the node is on. 0 would mean the node is the scene.
        int sceneDepth_ = 1;
        void calculateSceneDepth();

        bool graphTraverseFlag = false;



        void freeBody();

        /// rebuilds the internal body based on the current status of collision shapes on this node and child nodes. (be sure to update the children first!)
        void reBuildBody();

		void updateChildCollisionShapes(ea::vector<NewtonCollisionShape*>& enabledCollisionShapes);





        virtual void OnNodeSet(Node* node) override;

        virtual void OnSceneSet(Scene* scene) override;


        void HandleNodeAdded(StringHash event, VariantMap& eventData);
        void HandleNodeRemoved(StringHash event, VariantMap& eventData);


        //variables for deferered singular actions on the newtonbody in case it has not been created yet.
        void applyDefferedActions();

        //apply properties that have not had a chance to apply to body yet.
        void applyDefferedProperties();

        bool nextTransformNeeded_ = false;
        Matrix3x4 nextTransform_;

        bool nextPositionNeeded_ = false;
        Vector3 nextPosition_;

        bool nextOrientationNeeded_ = false;
        Quaternion nextOrientation_;

        bool nextLinearVelocityNeeded_ = false;
        Vector3 nextLinearVelocity_;
        bool nextLinearVelocityUseForces_ = true;
        bool nextAngularVelocityNeeded_ = false;
        Vector3 nextAngularVelocity_;
        bool nextImpulseNeeded_ = false;
        Vector3 nextImpulseWorldVelocity_;
        Vector3 nextImpulseLocalPos_;
        bool nextSleepStateNeeded_ = false;
        bool nextSleepState_ = false;


        //interpolation
        //void updateInterpolatedTransform();
        //Vector3 targetNodePos_;
        //Quaternion targetNodeRotation_;
        //Vector3 interpolatedNodePos_;
        //Quaternion interpolatedNodeRotation_;
        //float interpolationFactor_ = 1.0f;


		//cached states
		Vector3 lastAngularVelocity_;
		Vector3 lastLinearVelocity_;


        virtual void OnNodeSetEnabled(Node* node) override;

        /// Setting this to true will make the rigid body act as a root scene body with Inifite mass.
        void SetIsSceneRootBody(bool enable);
        bool GetIsSceneRootBody() const { return sceneRootBodyMode_; }

        virtual void OnMarkedDirty(Node* node) override;
    



    };


	class NewtonBodyNotifications : public ndBodyNotify
	{
	public:
		D_CLASS_REFLECTION(NewtonBodyNotifications);

		NewtonBodyNotifications();


		virtual void OnTransform(ndInt32 threadIndex, const ndMatrix& matrix);

		virtual void Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const;

		virtual void OnApplyExternalForce(ndInt32 threadIndex, ndFloat32 timestep);


		NewtonRigidBody* rigidBodyComponent_ = nullptr;
		Scene* scene_ = nullptr;
		NewtonPhysicsWorld* newtonPhysicsWorld_ = nullptr;
	};

	inline bool RigidBodySceneDepthCompare(const WeakPtr<NewtonRigidBody>& body1, const WeakPtr<NewtonRigidBody>& body2) {
        return (body1->GetSceneDepth() < body2->GetSceneDepth());
    }

}
