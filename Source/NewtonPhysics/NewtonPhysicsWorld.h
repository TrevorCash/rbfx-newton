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
#include "NewtonCollisionShape.h"

#include "ndNewton.h"




namespace Urho3D
{
	class NewtonModel;
	class Component;
    class NewtonCollisionShape;
    class NewtonRigidBody;
    class NewtonConstraint;
    class Sphere;
    class Ray;
    class BoundingBox;
    class NewtonMeshObject;
    class Context;

    static const Vector3 DEF_GRAVITY = Vector3(0, -9.81, 0);
    static const eastl::string DEF_PHYSICS_CATEGORY = "Physics";
    static const int DEF_PHYSICS_MAX_CONTACT_POINTS = 512;//maximum number of contacts per contact entry.





    

    struct PhysicsRayCastIntersection {
        ndBody* body_ = nullptr;
        ndShapeInstance* collision_ = nullptr;
		ndShapeInstance* subCollision_ = nullptr;
        float rayIntersectParameter_ = -1.0f;


        NewtonRigidBody* rigBody_ = nullptr;
        NewtonCollisionShape* collisionShape_ = nullptr;
        Vector3 rayIntersectWorldPosition_;
        Vector3 rayIntersectWorldNormal_;
        float rayDistance_ = -1.0f;
        Vector3 rayOriginWorld_;
    };

   /* URHONEWTON_API void PrintPhysicsRayCastIntersection(PhysicsRayCastIntersection& intersection);

    inline bool PhysicsRayCastIntersectionCompare(const PhysicsRayCastIntersection& intersect1, const PhysicsRayCastIntersection& intersect2) {
        return (intersect1.rayIntersectParameter_ < intersect2.rayIntersectParameter_);
    }
    struct PhysicsRayCastUserData {
        ea::vector<PhysicsRayCastIntersection> intersections;
        unsigned bodyIntersectionCounter_ = M_MAX_UNSIGNED;
    };
*/


	class NewtonWorldContactNotify : public ndContactNotify
	{
	public:
		NewtonWorldContactNotify();

		virtual ~NewtonWorldContactNotify();

		virtual void OnBodyAdded(ndBodyKinematic* const) const;

		virtual void OnBodyRemoved(ndBodyKinematic* const) const;

		virtual ndMaterial GetMaterial(const ndContact* const contact, const ndShapeInstance& shape1,  ndShapeInstance& shape2) const;

		virtual bool OnCompoundSubShapeOverlap(const ndContact* const contact, ndFloat32 timestep, const ndShapeInstance* const subShapeA, const ndShapeInstance* const subShapeB);

		virtual bool OnAabbOverlap(const ndContact* const contact, ndFloat32 timestep);

		virtual void OnContactCallback(ndInt32 threadIndex, const ndContact* const contact, ndFloat32 timestep);
	};





    class URHONEWTON_API NewtonPhysicsWorld : public Component
    {
        URHO3D_OBJECT(NewtonPhysicsWorld, Component);
    public:

        friend class NewtonCollisionShape;
        friend class NewtonCollisionShape_Geometry;
        friend class NewtonCollisionShape_ConvexDecompositionCompound;
        friend class NewtonCollisionShape_SceneCollision;
        friend class NewtonRigidBody;
        friend class NewtonConstraint;
        friend class NewtonModel;

        /// Construct.
        NewtonPhysicsWorld(Context* context);
        /// Destruct. Free the rigid body and geometries.
        ~NewtonPhysicsWorld() override;
        /// Register object factory.
        static void RegisterObject(Context* context);

        /// Return the internal Newton world.
        ndWorld* GetNewtonWorld() { return newtonWorld_; }


        /// Saves the NewtonWorld to a serializable newton file.
		void SerializeNewtonWorld(eastl::string fileName);

        /// Return a name for the currently used speed plugin (SSE, AVX, AVX2)
		eastl::string GetSolverPluginName();



        bool RigidBodyContainsPoint(NewtonRigidBody* rigidBody, const Vector3&worldPoint);
        /// Return rigid bodies by a ray query. bodies are returned in order from closest to farthest along the ray.
        void RayCast(
            eastl::vector<PhysicsRayCastIntersection>& intersections,
            const Ray& ray, float maxDistance = M_LARGE_VALUE,
            unsigned maxBodyIntersections = M_MAX_UNSIGNED,
            unsigned collisionMask = M_MAX_UNSIGNED);
        /// Return rigid bodies by a ray query.
        void RayCast(
			eastl::vector<PhysicsRayCastIntersection>& intersections,
            const Vector3& pointOrigin, const Vector3& pointDestination,
            unsigned maxBodyIntersections = M_MAX_UNSIGNED,
            unsigned collisionMask = M_MAX_UNSIGNED);


        /// Return rigid bodies by a sphere query.
        void GetRigidBodies(ea::vector<NewtonRigidBody*>& result, const Sphere& sphere, unsigned collisionMask = M_MAX_UNSIGNED);
        /// Return rigid bodies by a box query.
        void GetRigidBodies(ea::vector<NewtonRigidBody*>& result, const BoundingBox& box, unsigned collisionMask = M_MAX_UNSIGNED);
        /// Return rigid bodies by contact test with the specified body.
        void GetRigidBodies(ea::vector<NewtonRigidBody*>& result, const NewtonRigidBody* body);

    	void GetConnectedPhysicsComponents(NewtonRigidBody* rigidBody,
            ea::vector<NewtonRigidBody*>& rigidBodiesOUT,
            ea::vector<NewtonConstraint*>& constraintsOUT);



        /// Force the physics world to rebuild (will rebuild everything)
        void ForceRebuild() { WaitForUpdateFinished(); freePhysicsInternals(); rebuildDirtyPhysicsComponents(); }

        /// Force the physics world to update builds (will build only things that need built)
        void ForceBuild() {
            WaitForUpdateFinished();
            rebuildDirtyPhysicsComponents();
        }
        
        ///set the global force acting on all rigid bodies in the world this force is always the same regardless of physics world scale.
        void SetGravity(const Vector3& force);
        ///return global force acting on all rigid bodies
        Vector3 GetGravity() const;

        void SetTimeScale(float timescale){ timeScale_ = Max<float>(0.0f,timescale);}
        float GetTimeScale() const { return timeScale_; }
        

		NewtonRigidBody* GetSceneBody() const { return sceneBody_; }

		///set the scale of the debug renders for physics components.
		void SetDebugScale(float scale) { debugScale_ = scale; }

        ///waits until the asynchronous update has finished.
        void WaitForUpdateFinished();

        bool GetIsUpdating() { return isUpdating_; }

		///sets how many more steps to simulate going forward.  set numSteps < 0 for continued regular simulation.
		void SetRemainingSteps(int numSteps) { stepsRemaining_ = numSteps; }

		int GetRemainingSteps() const { return stepsRemaining_; }


        /// set how many iterations newton will run.
        void SetIterationCount(int numIterations);

        int GetIterationCount() const;
        /// set how many sub-updates to run vs the core update rate. must be 8, 4, 2, or 1
        void SetSubstepFactor(int numSubsteps);

        int GetSubstepFactor() const;
        /// set how many threads the newton can use.
        void SetThreadCount(int numThreads);

        int GetThreadCount() const;

        void Update(float timestep, bool isRootUpdate);

        void BuildAndUpdateNewtonModels();

        virtual void DrawDebugGeometry(DebugRenderer* debug, bool depthTest) override;

        void DrawDebugGeometry(DebugRenderer* debug, bool drawConstraints, bool drawContacts, bool drawRigidBodies, bool depthTest);


        bool isUpdating_ = false;

    protected:


        ///Global acceleration due to gravity
        Vector3 gravity_ = DEF_GRAVITY;

        /// number of thread to allow newton to use
        int newtonThreadCount_ = 16;
        /// number of iterations newton will internally use per substep
        int iterationCount_ = 4;
        /// number of substeps per scene subsystem update. (1,2,4,8)
        int subSteps_ = 2;

		float maxTimeStep_ = 1.0f / 60.0f;

		float timeStep_ = 0.0f;

        virtual void OnSceneSet(Scene* scene) override;

        void addCollisionShape(NewtonCollisionShape* collision);
        void removeCollisionShape(NewtonCollisionShape* collision);

        void addRigidBody(NewtonRigidBody* body);
        void removeRigidBody(NewtonRigidBody* body);

        void addConstraint(NewtonConstraint* constraint);
        void removeConstraint(NewtonConstraint* constraint);

        void addModel(NewtonModel* model);
        void removeModel(NewtonModel* model);

        void markRigidBodiesNeedSorted() { rigidBodyListNeedsSorted = true; }
        bool rigidBodyListNeedsSorted = true;

		eastl::vector<WeakPtr<NewtonCollisionShape>> collisionComponentList;
		eastl::vector<WeakPtr<NewtonRigidBody>> rigidBodyComponentList;
		eastl::vector<WeakPtr<NewtonConstraint>> constraintList;
        eastl::vector<WeakPtr<NewtonModel>> modelList;


        void freeWorld();

        void addToFreeQueue(ndBody* newtonBody);
        void addToFreeQueue(ndConstraint* newtonConstraint);
        void addToFreeQueue(ndShapeInstance* newtonShape);
        void addToFreeQueue(ndModel* newtonShape);



		eastl::vector<ndBody*> freeBodyQueue_;
		eastl::vector<ndConstraint*> freeConstraintQueue_;
		eastl::vector<ndShapeInstance*> freeShapeQueue_;
        eastl::vector<ndModel*> freeModelQueue_;


        void applyNewtonWorldSettings();

		NewtonWorldContactNotify* newtonContactNotify_ = nullptr;

        bool contactMapLocked_ = false;

        /// Step the simulation forward.
        void HandleSceneUpdate(StringHash eventType, VariantMap& eventData);

        void rebuildDirtyPhysicsComponents();
        bool simulationStarted_ = false;
		int stepsRemaining_ = -1;

        


        /// Internal newton world
        ndWorld* newtonWorld_ = nullptr;

        NewtonRigidBody* sceneBody_ = nullptr;

        float timeScale_ = 1.0f;

		float debugScale_ = 1.0f;
 

        ///convex casts
        //int DoNewtonCollideTest(const dFloat* const matrix, const NewtonCollision* shape);
		void GetBodiesInConvexCast(ea::vector<NewtonRigidBody*>& result, int numContacts);

        ///newton mesh caching
		ea::hash_map <StringHash, SharedPtr<NewtonMeshObject>> newtonMeshCache_;

        ///returns a unique key for looking up an exising NewtonMesh from the cache.
		static StringHash NewtonMeshKey(ea::string modelResourceName, int modelLodLevel, ea::string otherData);
        NewtonMeshObject* GetCreateNewtonMesh(StringHash urhoNewtonMeshKey);
        NewtonMeshObject* GetNewtonMesh(StringHash urhoNewtonMeshKey);
        

        void freePhysicsInternals();
};



	eastl::string NewtonThreadProfilerString(int threadIndex);


    //void Newton_PostUpdateCallback(const NewtonWorld* const world, dFloat timestep);


    /// newtwon body callbacks

    //void Newton_ApplyForceAndTorqueCallback(const NewtonBody* body, dFloat timestep, int threadIndex);
    //void Newton_SetTransformCallback(const NewtonBody* body, const dFloat* matrix, int threadIndex);
    //void Newton_DestroyBodyCallback(const NewtonBody* body);
    //unsigned Newton_WorldRayPrefilterCallback(const NewtonBody* const body, const NewtonCollision* const collision, void* const userData);
    //dFloat Newton_WorldRayCastFilterCallback(const NewtonBody* const body, const NewtonCollision* const collisionHit, const dFloat* const contact, const dFloat* const normal, dLong collisionID, void* const userData, dFloat intersetParam);

    ///newton joint callbacks
    //void Newton_JointDestructorCallback(const NewtonJoint* const joint);
    //void Newton_DestroyContactCallback(const NewtonWorld* const newtonWorld, NewtonJoint* const contact);


    /// newton material callbacks
 /*   void Newton_ProcessContactsCallback(const NewtonJoint* contactJoint, dFloat timestep, int threadIndex);
    int Newton_AABBOverlapCallback(const NewtonJoint* const contactJoint, dFloat timestep, int threadIndex);
    int Newton_AABBCompoundOverlapCallback(const NewtonJoint* const contact, dFloat timestep, const NewtonBody* const body0, const void* const collisionNode0, const NewtonBody* const body1, const void* const collisionNode1, int threadIndex);

    int Newton_WakeBodiesInAABBCallback(const NewtonBody* const body, void* const userData);

*/





	URHONEWTON_API void  GetRootRigidBodies(eastl::vector<NewtonRigidBody*>& rigidBodies, Node* node, bool includeScene);

	URHONEWTON_API void NewtonCompoundGetSubShapes(ndShapeCompound* compound, ea::vector<ndShapeInstance*>& subShapes);


	URHONEWTON_API void  GetNextChildRigidBodies(eastl::vector<NewtonRigidBody*>& rigidBodies, Node* node);
	URHONEWTON_API void  GetAloneCollisionShapes(eastl::vector<NewtonCollisionShape*>& colShapes, Node* startingNode, bool includeStartingNodeShapes);

	URHONEWTON_API void  CalculateRigidBodyGroupFusedVelocities(ea::vector<NewtonRigidBody*>& rigidBodies, Matrix3x4 worldReferenceFrame, Vector3& worldVelocity, Vector3& worldAngularVelocity);

	///Parallel Axis Theorem:
	URHONEWTON_API Matrix3 MoveInertiaMatrix(Matrix3 matrix, const float mass, const Vector3& delta);

    URHONEWTON_API void  RebuildPhysicsNodeTree(Node* node);

    URHONEWTON_API unsigned CollisionLayerAsBit(unsigned layer);
    /// Register Physics library objects.
    URHONEWTON_API void RegisterNewtonPhysicsLibrary(Context* context);
}
