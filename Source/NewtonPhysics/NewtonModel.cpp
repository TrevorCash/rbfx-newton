#include "NewtonModel.h"

#include "NewtonConstraint.h"
#include "NewtonRevoluteJoint.h"
#include "NewtonPhysicsWorld.h"
#include "NewtonRigidBody.h"

#include "Urho3D/Scene/Scene.h"
#include "Urho3D/SystemUI/SystemUI.h"
#include "Urho3D/SystemUI/Imgui_eigen.h"

#include "UrhoNewtonConversions.h"
#include "ndNewton.h"


namespace  Urho3D
{
	void ChainJacobian::ToEigenMatrix_Full(Eigen::MatrixXd& J)
	{
        int numJoints = J_v.size();
        J.resize(6, numJoints);
        for (int c = 0; c < numJoints; c++)
        {
            J(0, c) = J_v[c].x_;
            J(1, c) = J_v[c].y_;
            J(2, c) = J_v[c].z_;

            J(3, c) = J_w[c].x_;
            J(4, c) = J_w[c].y_;
            J(5, c) = J_w[c].z_;
        }
	}
    void ChainJacobian::ToEigenMatrix_Linear(Eigen::MatrixXd& J)
    {
        int numJoints = J_v.size();
        J.resize(3, numJoints);
        for (int c = 0; c < numJoints; c++)
        {
            J(0, c) = J_v[c].x_;
            J(1, c) = J_v[c].y_;
            J(2, c) = J_v[c].z_;
        }
    }
    void ChainJacobian::ToEigenMatrix_Angular(Eigen::MatrixXd& J)
    {
        int numJoints = J_v.size();
        J.resize(3, numJoints);
        for (int c = 0; c < numJoints; c++)
        {
            J(0, c) = J_w[c].x_;
            J(1, c) = J_w[c].y_;
            J(2, c) = J_w[c].z_;
        }
    }



	void NewtonModel::RegisterObject(Context* context)
	{
		context->RegisterFactory<NewtonModel>(DEF_PHYSICS_CATEGORY.c_str());

	}

	NewtonModel::NewtonModel(Context* context) : Component(context)
	{
	}

	void NewtonModel::GrowFrom(WeakPtr<NewtonConstraint> constraint)
	{

        bodies.clear();
        constraints.clear();

        if (!constraint)
            return;

        constraint->model_ = this;

        physicsWorld_->GetConnectedPhysicsComponents(constraint->GetOwnBody(), bodies, constraints);

        for (auto body : bodies)
            body->model_ = this;

        for (auto constraint : constraints)
            constraint->model_ = this;

	}

	void NewtonModel::Grow()
	{

        GrowFrom(constraints.front());
	}



    void NewtonModel::freeModel()
    {
        if (newtonModel_ != nullptr) 
        {
            physicsWorld_->removeNewtonModelQueue_.push_back(newtonModel_);
            newtonModel_ = nullptr;
        }
    }

    void NewtonModel::reBuild()
    {
        freeModel();
        newtonModel_ = new NewtonModelHandler();
        newtonModel_->model_ = this;
        physicsWorld_->GetNewtonWorld()->AddModel(newtonModel_);
    }


    void NewtonModel::CalculateChainJabobian(ea::vector<NewtonRevoluteJoint*>& constraintChain,
        ChainJacobian& J)
    {
        //compute the Jacobian From end to base
        J.J_v.resize(constraintChain.size());
        J.J_w.resize(constraintChain.size());

        Matrix3x4 rootTransform = constraintChain[0]->GetOwnWorldFrame();
        Vector3 rootWorldVel = constraintChain[0]->GetOwnWorldFrameVel();
        Vector3 rootWorldOmega = constraintChain[0]->GetOwnWorldFrameOmega();
        Matrix3x4 endEffectorWorld = constraintChain[constraintChain.size()-1]->GetOwnWorldFrame();
        Matrix3x4 endEffectorRelRoot = rootTransform.Inverse() * endEffectorWorld;
        Vector3 hingeLocalRotationAxis = NewtonRevoluteJoint::LocalHingeAxis();
        Vector3 d_n_0 = endEffectorRelRoot.Translation();

        for (int i = 0; i < constraintChain.size(); i++)
        {
            Matrix3x4 rootSpace = rootTransform.Inverse() * constraintChain[i]->GetOwnWorldFrame();
            Matrix3 r_i_0 = rootSpace.RotationMatrix();
            Vector3 d_i_0 = rootTransform.Inverse() * constraintChain[i]->GetOwnWorldFrame().Translation();
            J.J_v[i] = (r_i_0 * hingeLocalRotationAxis).CrossProduct(d_n_0 - d_i_0);
        }
        for (int i = 0; i < constraintChain.size(); i++)
        {
            Matrix3x4 rootSpace = rootTransform.Inverse() * constraintChain[i]->GetOwnWorldFrame();
            Matrix3 r_i_0 = rootSpace.RotationMatrix();
            J.J_w[i] = (r_i_0 * hingeLocalRotationAxis);
        }
    }

    void NewtonModel::SolveForJointVelocities(ChainJacobian& J, ea::vector<NewtonRevoluteJoint*>& constraintChain, Vector3 endVelWorld,
	    Vector3 endOmegaWorld, ea::vector<float>& velocitiesOut)
	{
        int numJoints = J.J_v.size();
        Eigen::MatrixXd jeig;
        J.ToEigenMatrix_Full(jeig);


        Eigen::VectorXd EndTargetVel(6);
        EndTargetVel(0) = endVelWorld.x_;
        EndTargetVel(1) = endVelWorld.y_;
        EndTargetVel(2) = endVelWorld.z_;
        EndTargetVel(3) = endOmegaWorld.x_;
        EndTargetVel(4) = endOmegaWorld.y_;
        EndTargetVel(5) = endOmegaWorld.z_;


        Eigen::VectorXd SolvedQ_vel = jeig.completeOrthogonalDecomposition().solve(EndTargetVel);// J_inv* target_vel;


        velocitiesOut.clear();
        for(int i = 0; i < numJoints; i++)
			velocitiesOut.push_back(SolvedQ_vel(i));

	}

    void NewtonModel::ComputeJointTorquesForEndEffector(ChainJacobian& J, ea::vector<NewtonRevoluteJoint*>& constraintChain,
	    Vector3 endForceRobotSpace, Vector3 endTorqueRobotSpace, ea::vector<float>& torquesOut)
	{
        int numJoints = J.J_v.size();
        Eigen::MatrixXd jeig;
        J.ToEigenMatrix_Full(jeig);

        Eigen::VectorXd EndTargetWrench(6);
        EndTargetWrench(0) = endForceRobotSpace.x_;
        EndTargetWrench(1) = endForceRobotSpace.y_;
        EndTargetWrench(2) = endForceRobotSpace.z_;
        EndTargetWrench(3) = endTorqueRobotSpace.x_;
        EndTargetWrench(4) = endTorqueRobotSpace.y_;
        EndTargetWrench(5) = endTorqueRobotSpace.z_;


        Eigen::VectorXd SolvedQ_torques = jeig.transpose() * EndTargetWrench;

        torquesOut.clear();
        for (int i = 0; i < numJoints; i++)
            torquesOut.push_back(SolvedQ_torques(i));
	}


    void NewtonModel::ComputeCounterGravitationalTorque(ea::vector<NewtonRevoluteJoint*>& constraintChain, ea::vector<float>& torquesOut)
    {
        for (int i = 0; i < constraintChain.size(); i++)
        {
            //compute effective current inertia frame on the remaining links
            float totalMass = 0.0f;
            Vector3 totalCOM;
            
            for (int j = i; j < constraintChain.size(); j++)
            {
                float mass = constraintChain[j]->GetOtherBody()->GetEffectiveMass();
                Matrix3x4 com = constraintChain[j]->GetOtherBody()->GetCOMWorldTransform();

                totalCOM += mass * com.Translation();
                totalMass += mass;
            }
            totalCOM *= 1 / (totalMass);


            Vector3 forceDueToGravity(0, totalMass * -9.81f, 0);
            Vector3 counterTorque = (totalCOM - constraintChain[i]->GetOwnWorldFrame().Translation()).CrossProduct(forceDueToGravity)*-1.0f;
            
            Vector3 constraintWorldHingeAxis = constraintChain[i]->GetOwnWorldFrame().RotationMatrix() * constraintChain[i]->LocalHingeAxis();
            float jointTorque = counterTorque.ProjectOntoAxis(constraintWorldHingeAxis);
            torquesOut.push_back(jointTorque);
        }

    }

    void NewtonModel::ComputeEndEffectorManipulabilityElipse(ChainJacobian& J, Vector3& v1, Vector3& v2, Vector3& v3, Vector3& lengths)
    {
        Eigen::EigenSolver<Eigen::MatrixXd> eigenSolver;
        Eigen::MatrixXd Jac_v;
        J.ToEigenMatrix_Linear(Jac_v);


        Eigen::MatrixXd A = Jac_v * Jac_v.transpose();
        eigenSolver.compute(A, true);

        
        auto eigenVectors = eigenSolver.eigenvectors();
        auto eigenValues = eigenSolver.eigenvalues();

        v1.x_ = eigenVectors(0, 0).real();
        v1.y_ = eigenVectors(1, 0).real();
        v1.z_ = eigenVectors(2, 0).real();
        v1.Normalize();

        v2.x_ = eigenVectors(0, 1).real();
        v2.y_ = eigenVectors(1, 1).real();
        v2.z_ = eigenVectors(2, 1).real();
        v2.Normalize();

        v3.x_ = eigenVectors(0, 2).real();
        v3.y_ = eigenVectors(1, 2).real();
        v3.z_ = eigenVectors(2, 2).real();
        v3.Normalize();

        lengths.x_ = Sqrt(eigenValues(0).real());
        lengths.y_ = Sqrt(eigenValues(1).real());
        lengths.z_ = Sqrt(eigenValues(2).real());


        //ui::Text("Jac_v");
        //ui::EIGEN_MATRIX_XD_TABLE("jac_v", Jac_v);

        //ui::Text("A");
        //ui::EIGEN_MATRIX_XD_TABLE("A", A);

        //ui::Text("EigenVectors");
        //ui::EIGEN_MATRIX_XCD_TABLE("eigvectors", eigenVectors);

        //ui::Text("v1 dot v2: %f", v1.DotProduct(v2));
        //ui::Text("v1 dot v3: %f", v1.DotProduct(v3));
        //ui::Text("v2 dot v3: %f", v2.DotProduct(v3));

        //ui::Text("EigenValues");
        //ui::EIGEN_VECTOR_XCD_TABLE("eigvals", eigenValues);

    }




    void NewtonModel::PreSolveComputations(ndWorld* const world, ndFloat32 timestep)
    {
       // NewtonRigidBody* base = nullptr;



       // for(auto* body : bodies)
       // {
       //     if (body->GetNode()->GetName() == "RobotBase")
       //         base = body;
       // }



       // ndBodyKinematic* sceneBody = base->GetScene()->GetComponent<NewtonRigidBody>()->GetNewtonBody()->GetAsBodyKinematic();

       // //ndJointKinematicController* endEffector = new ndJointKinematicController(sceneBody, ,)
       // //end

       // 

       // ndSkeletonContainer* const skeleton = base->GetNewtonBody()->GetAsBodyKinematic()->GetSkeleton();
       // ndSkeletonImmediateSolver solve;
       // solve.Solve(skeleton, world, timestep);

       // // use solver result to set joint motors
       // //for(int i = 0; i < solve.m_internalForces.GetCount(); i++)
       // //{
       //     ndVector angular = solve.m_internalForces[0].m_angular;
       //     ndVector linear = solve.m_internalForces[0].m_linear;

       //     ea::vector<NewtonConstraint*> constraint;
       //     base->GetConnectedContraints(constraint);

       //     dynamic_cast<NewtonRevoluteJoint*>(constraint[0])->SetCommandedTorque(angular.GetX());
       //     


       //// }



        //ui::Begin("Joint Torques");
        //ui::BeginTable("Torques", 6);
        //for (int i = 0; i < constraints.size(); i++)
        //{
        //    ui::TableNextColumn();
        //    ui::Text("Own x %f", constraints[i]->GetOwnTorque().x_);
        //    ui::Text("Own y %f", constraints[i]->GetOwnTorque().y_);
        //    ui::Text("Own z %f", constraints[i]->GetOwnTorque().z_);
        //    ui::Text("Other x %f", constraints[i]->GetOtherTorque().x_);
        //    ui::Text("Other y %f", constraints[i]->GetOtherTorque().y_);
        //    ui::Text("Other z %f", constraints[i]->GetOtherTorque().z_);
        //}

        //ui::EndTable();
        //ui::End();



    }

    void NewtonModel::UpdateBoundingBox()
    {
        boundingBox_.Clear();
        for(auto body : bodies)
        {
            ndVector p0;
            ndVector p1;

            body->GetNewtonBody()->GetAABB(p0,p1);
            boundingBox_.Merge(NewtonToUrhoVec3(p0));
            boundingBox_.Merge(NewtonToUrhoVec3(p1));
        }
    }

    inline void NewtonModel::OnSceneSet(Scene* scene)
	{
		Component::OnSceneSet(scene);
        if (scene)
        {
            ///auto create physics world
            physicsWorld_ = WeakPtr<NewtonPhysicsWorld>(scene->GetComponent<NewtonPhysicsWorld>());

            physicsWorld_->addModel(this);
            scene->AddListener(this);
        }
        else
        {
            if (!physicsWorld_.Expired())
				physicsWorld_->removeModel(this);
        }
	}




















    NewtonModelHandler::NewtonModelHandler() : ndModel()
    {
    }

    void NewtonModelHandler::Update(ndWorld* const world, ndFloat32 timestep)
    {
        //ndModel::Update(world, timestep);
        model_->PreSolveComputations(world, timestep);


    }

    void NewtonModelHandler::PostUpdate(ndWorld* const world, ndFloat32 timestep)
    {
        //ndModel::PostUpdate(world, timestep);
    }

    void NewtonModelHandler::PostTransformUpdate(ndWorld* const world, ndFloat32 timestep)
    {
        //ndModel::PostTransformUpdate(world, timestep);
    }

















}

