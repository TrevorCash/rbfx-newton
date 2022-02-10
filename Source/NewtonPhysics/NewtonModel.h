#pragma once


#include "ndModel.h"
#include "Urho3D/Scene/Component.h"
#include "UrhoNewtonApi.h"
#include "eigen/Eigen/Dense"

class ndModel;

namespace Urho3D
{
	class NewtonHingeConstraint;
	class NewtonModelHandler;
	class NewtonConstraint;


    //6xN Jacobian
    class URHONEWTON_API ChainJacobian
    {
    public:
        ea::vector<Vector3> J_v;
        ea::vector<Vector3> J_w;

        void ToEigenMatrix(Eigen::MatrixXd& J);
    };





	///newton model component representing collection of bodies and joints.
    ///exists only on scene node
    ///created and managed by the NewtonWorld Component
    class URHONEWTON_API NewtonModel : public Component
    {

    public:
        friend class NewtonPhysicsWorld;
        friend class NewtonConstraint;
        friend class NewtonRigidBody;

	    URHO3D_OBJECT(NewtonModel, Component);


        static void RegisterObject(Context* context);

        NewtonModel(Context* context);

        void GrowFrom(NewtonConstraint* constraint);
        void Grow();

        bool IsEmpty() {
            return constraints.empty();
        }

        void MarkDirty() { dirty_ = true; }
        bool GetDirty() const { return dirty_; }


        bool GetShortestChain(NewtonConstraint* rootConstraint, NewtonConstraint* endConstraint, ea::vector<NewtonConstraint*>& chain)
        {
            return false;
        }


        //return the jacobian for the constraintChain
        void CalculateChainJabobian(ea::vector<NewtonHingeConstraint*>& constraintChain, ChainJacobian& J);


        //constraintChain from base to end.
        void SolveForJointVelocities(ChainJacobian& J, ea::vector<NewtonHingeConstraint*>& constraintChain,
            Vector3 endVelWorld, Vector3 endOmegaWorld, ea::vector<float>& velocitiesOut);

        //constraintChain from base to end.
        void SolveForJointTorques(ChainJacobian& J, ea::vector<NewtonHingeConstraint*>& constraintChain,
            Vector3 endForceWorld, Vector3 endTorqueWorld, ea::vector<float>& torquesOut);

        //void SolveForEndVelocity();
        //void SolveForEndForces();


        void PreSolveComputations(ndWorld* const world, ndFloat32 timestep);

    protected:
	    void OnSceneSet(Scene* scene) override;
        void freeModel();
        void reBuild();

        ea::vector<NewtonRigidBody*> bodies;
        ea::vector<NewtonConstraint*> constraints;

        NewtonModelHandler* newtonModel_ = nullptr;
        bool dirty_ = true;

        WeakPtr<NewtonPhysicsWorld> physicsWorld_ = nullptr;
    };



    
    class NewtonModelHandler : public ndModel
    {
    public:
    public:
        D_CLASS_REFLECTION(NewtonModelHandler);
        NewtonModelHandler();

        friend class NewtonModel;
    protected:
        inline void Update(ndWorld* const world, ndFloat32 timestep) override;
        inline void PostUpdate(ndWorld* const world, ndFloat32 timestep) override;
        inline void PostTransformUpdate(ndWorld* const world, ndFloat32 timestep) override;

        NewtonModel* model_ = nullptr;
    };


}
