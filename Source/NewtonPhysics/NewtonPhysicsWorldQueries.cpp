#include "NewtonPhysicsWorld.h"
#include "UrhoNewtonConversions.h"
#include "NewtonRigidBody.h"
#include "NewtonCollisionShape.h"
#include "NewtonConstraint.h"


#include "Urho3D/Math/Sphere.h"
#include "Urho3D/IO/Log.h"




namespace Urho3D {

    bool NewtonPhysicsWorld::RigidBodyContainsPoint(NewtonRigidBody* rigidBody, const Vector3&worldPoint)
    {
		return false;
        //ndVector contact;
        //ndVector normal;

        //ndShape* effectiveCollision = rigidBody->GetEffectiveNewtonShape();

        //if (!effectiveCollision)
        //    return false;

        //ndMatrix collisionMatrix;
        //NewtonCollisionGetMatrix(effectiveCollision, &collisionMatrix[0][0]);

        ////#todo double check the matrix here may need tweeking to get the true transform of the compound:
        //int res = NewtonCollisionPointDistance(newtonWorld_,
        //    &UrhoToNewton(worldPoint)[0],
        //    rigidBody->GetEffectiveNewtonShape(),
        //    &UrhoToNewton((rigidBody->GetWorldTransform()) * Matrix3x4(NewtonToUrhoMat4(collisionMatrix)))[0][0], &contact[0], &normal[0], 0);


        //return !res;
    }

    void NewtonPhysicsWorld::GetRigidBodies(ea::vector<NewtonRigidBody*>& result, const Sphere& sphere, unsigned collisionMask /*= M_MAX_UNSIGNED*/)
    {
        //#todo use collision mask

        //Matrix3x4 mat;
        //mat.SetTranslation(sphere.center_);

        //NewtonCollision* newtonShape = UrhoShapeToNewtonCollision(newtonWorld_, sphere, false);
        //int numContacts = DoNewtonCollideTest(&UrhoToNewton(mat)[0][0], newtonShape);

        //GetBodiesInConvexCast(result, numContacts);

        //NewtonDestroyCollision(newtonShape);
    }

    void NewtonPhysicsWorld::GetRigidBodies(ea::vector<NewtonRigidBody*>& result, const BoundingBox& box, unsigned collisionMask /*= M_MAX_UNSIGNED*/)
    {
        ////#todo use collision mask
        //Matrix3x4 mat;
        //mat.SetTranslation(box.Center());

        //NewtonCollision* newtonShape = UrhoShapeToNewtonCollision(newtonWorld_, box, false);
        //int numContacts = DoNewtonCollideTest(&UrhoToNewton(mat)[0][0], newtonShape);

        //GetBodiesInConvexCast(result, numContacts);
        //NewtonDestroyCollision(newtonShape);
    }


    void NewtonPhysicsWorld::GetRigidBodies(ea::vector<NewtonRigidBody*>& result, const NewtonRigidBody* body)
    {
        //dMatrix mat;
        //NewtonBodyGetMatrix(body->GetNewtonBody(), &mat[0][0]);
        //NewtonCollision* newtonShape = body->GetEffectiveNewtonShape();
        //int numContacts = DoNewtonCollideTest(&mat[0][0], newtonShape);

        //GetBodiesInConvexCast(result, numContacts);
    }

    void NewtonPhysicsWorld::GetConnectedPhysicsComponents(NewtonRigidBody* rigidBody,
        ea::vector<NewtonRigidBody*>& rigidBodiesOUT,
        ea::vector<NewtonConstraint*>& constraintsOUT)
    {

        ea::vector<NewtonRigidBody*> bodyQueue;
        bodyQueue.push_front(rigidBody);

        while (bodyQueue.size())
        {
            if (!bodyQueue.front()->graphTraverseFlag)
            {
                bodyQueue.front()->graphTraverseFlag = true;
                rigidBodiesOUT.push_back(bodyQueue.front());


                ea::vector<NewtonConstraint*> constraintsHere;
                rigidBody->GetConnectedContraints(constraintsHere);
                ea::vector<NewtonRigidBody*> bodiesHere;
                rigidBody->GetConnectedBodies(bodiesHere);

                for (auto* constraint : constraintsHere)
                {
                    if (!constraint->graphTraverseFlag)
                    {
                        constraint->graphTraverseFlag = true;
                        constraintsOUT.push_back(constraint);
                    }
                }

                for (auto* body : bodiesHere)
                {
                    if (!body->graphTraverseFlag)
                        bodyQueue.push_front(body);
                }

            }
            bodyQueue.pop_back();
        }

        //restore flags.
        for (auto* body : rigidBodiesOUT)
            body->graphTraverseFlag = false;

        for (auto* constraint : constraintsOUT)
            constraint->graphTraverseFlag = false;
    }


    //int NewtonPhysicsWorld::DoNewtonCollideTest(const ndFloat32* const matrix, const NewtonCollision* shape)
    //{
    //    //URHO3D_LOGINFO("DoNewtonCollideTest needs more testing.");
    //    //return  NewtonWorldCollide(newtonWorld_,
    //    //    matrix, shape, nullptr,
    //    //    Newton_WorldRayPrefilterCallback, convexCastRetInfoArray,
    //    //    convexCastRetInfoSize_, 0);

    //}
    void NewtonPhysicsWorld::GetBodiesInConvexCast(ea::vector<NewtonRigidBody*>& result, int numContacts)
    {
        ////iterate though contacts.
        //for (int i = 0; i < numContacts; i++) {

        //    if (convexCastRetInfoArray[i].m_hitBody != nullptr) {

        //        void* userData = NewtonBodyGetUserData(convexCastRetInfoArray[i].m_hitBody);
        //        if (userData != nullptr)
        //        {
        //            NewtonRigidBody* rigBody = static_cast<NewtonRigidBody*>(userData);
        //            result.push_back(rigBody);
        //        }
        //    }
        //}
    }
}

