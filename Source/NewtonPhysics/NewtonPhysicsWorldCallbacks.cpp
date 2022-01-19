#include "NewtonPhysicsWorld.h"
#include "UrhoNewtonConversions.h"
#include "NewtonRigidBody.h"


#include "Urho3D/Core/Profiler.h"
#include "Urho3D/Scene/Component.h"
#include "Urho3D/Scene/Scene.h"
#include "Urho3D/IO/Log.h"


#include "ndNewton.h"

namespace Urho3D {


	class NewtonContactNotifications : public ndContactNotify
	{
	public:
		virtual void OnBodyAdded(ndBodyKinematic* const) const
		{

		}

		virtual void OnBodyRemoved(ndBodyKinematic* const) const
		{
		}

		virtual ndMaterial GetMaterial(const ndContact* const, const ndShapeInstance&, const ndShapeInstance&) const
		{
			return ndMaterial();
		}

		//bool OnCompoundSubShapeOverlap(const ndContact* const contact, ndFloat32 timestep, const ndShapeInstance* const subShapeA, const ndShapeInstance* const subShapeB);
		bool OnCompoundSubShapeOverlap(const ndContact* const, ndFloat32, const ndShapeInstance* const, const ndShapeInstance* const)
		{
			return true;
		}

		//virtual bool OnAabbOverlap(const ndContact* const contact, ndFloat32 timestep)
		virtual bool OnAabbOverlap(const ndContact* const, ndFloat32)
		{
			return true;
		}

		virtual void OnContactCallback(ndInt32 thread, const ndContact* const contact, ndFloat32 timestep)
		{
			URHO3D_PROFILE_THREAD(NewtonThreadProfilerString(thread).c_str());

			//Get handles To NewtonBodies and RigidBody Components.
			const ndBodyKinematic* const body0 = contact->GetBody0();
			const ndBodyKinematic* const body1 = contact->GetBody1();

		}
	};


	void NewtonBodyNotifications::OnTransform(ndInt32 threadIndex, const ndMatrix& matrix)
	{
		
	}

	void NewtonBodyNotifications::Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const
	{
		
	}

	void NewtonBodyNotifications::OnApplyExternalForce(ndInt32 threadIndex, ndFloat32 timestep)
	{
		
	}

   // void Newton_ApplyForceAndTorqueCallback(const NewtonBody* body, dFloat timestep, int threadIndex)
   // {
   //     //URHO3D_PROFILE_THREAD(NewtonThreadProfilerString(threadIndex).c_str());
   //     URHO3D_PROFILE_FUNCTION()


   //     Vector3 netForce;
   //     Vector3 netTorque;
   //     NewtonRigidBody* rigidBodyComp = nullptr;

   //     rigidBodyComp = static_cast<NewtonRigidBody*>(NewtonBodyGetUserData(body));

   //     if (rigidBodyComp == nullptr)
   //         return;

   //     rigidBodyComp->GetForceAndTorque(netForce, netTorque);

   //     Vector3 gravityForce;
   //     if (rigidBodyComp->GetScene() != nullptr)//on scene destruction sometimes this is null so check...
   //     {
   //         NewtonPhysicsWorld* physicsWorld = rigidBodyComp->GetScene()->GetComponent<NewtonPhysicsWorld>();
			//if (physicsWorld != nullptr)
			//{
			//	gravityForce = physicsWorld->GetGravity() * rigidBodyComp->GetEffectiveMass();



			//	netForce += gravityForce;



			//	//apply forces and torques scaled with the physics world scale accourdingly.
			//	NewtonBodySetForce(body, &UrhoToNewton(netForce)[0]);
			//	NewtonBodySetTorque(body, &UrhoToNewton(netTorque)[0]);
			//}

   //     }
   // }


   // void Newton_SetTransformCallback(const NewtonBody* body, const dFloat* matrix, int threadIndex)
   // {
   //     NewtonRigidBody* rigBody = static_cast<NewtonRigidBody*>(NewtonBodyGetUserData(body));
   //     if(rigBody)
   //         rigBody->MarkInternalTransformDirty();
   // }


   // void Newton_DestroyBodyCallback(const NewtonBody* body)
   // {

   // }



   // dFloat Newton_WorldRayCastFilterCallback(const NewtonBody* const body,
   //     const NewtonCollision* const collisionHit, const dFloat* const contact,
   //     const dFloat* const normal, dLong collisionID, void* const userData, dFloat intersetParam)
   // {
   //     PhysicsRayCastUserData*  data = (PhysicsRayCastUserData*)userData;

   //     PhysicsRayCastIntersection intersection;
   //     intersection.body_ = (NewtonBody*)body;
   //     intersection.collision_ = (NewtonCollision*)collisionHit;
   //     intersection.rayIntersectParameter_ = intersetParam;
   //     intersection.rayIntersectWorldPosition_ = NewtonToUrhoVec3(dVector(contact));
   //     intersection.rayIntersectWorldNormal_ = NewtonToUrhoVec3(dVector(normal));
   //     intersection.rigBody_ = (NewtonRigidBody*)NewtonBodyGetUserData(body);
   //     intersection.collisionShape_ = (NewtonCollisionShape*)NewtonCollisionGetUserData(collisionHit);
   //     data->intersections.push_back(intersection);
   //     data->bodyIntersectionCounter_--;

   //     URHO3D_LOGINFO("RayIntersection: " + Urho3D::ToString((void*)collisionHit) + ", " + ea::to_string(collisionID));

   //     if (data->bodyIntersectionCounter_ > 0) {
   //         //continue
   //         return 1.0f;
   //     }
   //     else
   //         return 0.0f;
   // }




   // unsigned Newton_WorldRayPrefilterCallback(const NewtonBody* const body, const NewtonCollision* const collision, void* const userData)
   // {
   //     ///no filtering right now.
   //     return 1;///?
   // }





   // void Newton_DestroyContactCallback(const NewtonWorld* const newtonWorld, NewtonJoint* const contact)
   // {
   //     if (NewtonJointGetUserData(contact)) {
   //       
   //     }

   // }











  //  void Newton_ProcessContactsCallback(const NewtonJoint* contactJoint, dFloat timestep, int threadIndex)
  //  {
		//
		//


  //      //URHO3D_PROFILE_THREAD(NewtonThreadProfilerString(threadIndex).c_str());
  //      URHO3D_PROFILE_FUNCTION();

  //      //Get handles To NewtonBodies and RigidBody Components.
  //      const NewtonBody* const body0 = NewtonJointGetBody0(contactJoint);
  //      const NewtonBody* const body1 = NewtonJointGetBody1(contactJoint);

  //      NewtonRigidBody* rigBody0 = static_cast<NewtonRigidBody*>(NewtonBodyGetUserData(body0));
  //      NewtonRigidBody* rigBody1 = static_cast<NewtonRigidBody*>(NewtonBodyGetUserData(body1));

		//if (!rigBody0 || !rigBody1) {
		//	URHO3D_LOGINFO("Newton_ProcessContactsCallback missed due to rigBody null.");
		//	return;
		//}

  //      if (!rigBody0->GetGenerateContacts() || !rigBody1->GetGenerateContacts())
  //          return;

  //      


  //      NewtonPhysicsWorld* physicsWorld = rigBody0->GetPhysicsWorld();



  //      int contactIdx = 0;

  //      for (void* contact = NewtonContactJointGetFirstContact(contactJoint); contact; contact = NewtonContactJointGetNextContact(contactJoint, contact)) {


  //          NewtonMaterial* const material = NewtonContactGetMaterial(contact);

  //          NewtonCollision* shape0 = NewtonMaterialGetBodyCollidingShape(material, body0);
  //          NewtonCollision* shape1 = NewtonMaterialGetBodyCollidingShape(material, body1);


  //          NewtonCollisionShape* colShape0 = static_cast<NewtonCollisionShape*>(NewtonCollisionGetUserData(shape0));
  //          NewtonCollisionShape* colShape1 = static_cast<NewtonCollisionShape*>(NewtonCollisionGetUserData(shape1));




  //          {
  //              //get contact geometric info for the contact struct
  //              dVector pos, force, norm, tan0, tan1;
  //              NewtonMaterialGetContactPositionAndNormal(material, body0, &pos[0], &norm[0]);
  //              NewtonMaterialGetContactTangentDirections(material, body0, &tan0[0], &tan1[0]);
  //              NewtonMaterialGetContactForce(material, body0, &force[0]);


  //              //#todo debugging
  //              //GetSubsystem<VisualDebugger>()->AddCross(contactEntry->contactPositions[contactIdx], 0.1f, Color::BLUE, true);

  //              contactIdx++;
  //          }


  //          float staticFriction0 = colShape0->GetStaticFriction();
  //          float kineticFriction0 = colShape0->GetKineticFriction();
  //          float elasticity0 = colShape0->GetElasticity();
  //          float softness0 = colShape0->GetSoftness();

  //          float staticFriction1 = colShape1->GetStaticFriction();
  //          float kineticFriction1 = colShape1->GetKineticFriction();
  //          float elasticity1 = colShape1->GetElasticity();
  //          float softness1 = colShape1->GetSoftness();


  //          float finalStaticFriction = Max(staticFriction0, staticFriction1);
  //          float finalKineticFriction = Max(kineticFriction0, kineticFriction1);
  //          float finalElasticity = Min(elasticity0, elasticity1);
  //          float finalSoftness = Max(softness0, softness1);

  //          //apply material settings to contact.
  //          NewtonMaterialSetContactFrictionCoef(material, finalStaticFriction, finalKineticFriction, 0);
  //          NewtonMaterialSetContactElasticity(material, finalElasticity);
  //          NewtonMaterialSetContactSoftness(material, finalSoftness);

  //          if (rigBody0->GetTriggerMode() || rigBody1->GetTriggerMode()) {
  //              NewtonContactJointRemoveContact(contactJoint, contact);
  //              continue;
  //          }
  //      }


  //  }






  //  int Newton_AABBOverlapCallback(const NewtonJoint* const contactJoint, dFloat timestep, int threadIndex)
  //  {
  //      //URHO3D_PROFILE_THREAD(NewtonThreadProfilerString(threadIndex).c_str());
  //      URHO3D_PROFILE_FUNCTION();


  //      const NewtonBody* const body0 = NewtonJointGetBody0(contactJoint);
  //      const NewtonBody* const body1 = NewtonJointGetBody1(contactJoint);

  //      NewtonRigidBody* rigBody0 = static_cast<NewtonRigidBody*>(NewtonBodyGetUserData(body0));
  //      NewtonRigidBody* rigBody1 = static_cast<NewtonRigidBody*>(NewtonBodyGetUserData(body1));

		//if (!rigBody0 || !rigBody1)
		//	return 0;

  //      NewtonPhysicsWorld* physicsWorld = rigBody0->GetPhysicsWorld();



  //      bool res;
  //      NewtonWorldCriticalSectionLock(physicsWorld->GetNewtonWorld(), threadIndex);

  //      res = rigBody1->CanCollideWith(rigBody0);


  //      NewtonWorldCriticalSectionUnlock(physicsWorld->GetNewtonWorld());
  //      return res;
  //  }


  //  int Newton_AABBCompoundOverlapCallback(const NewtonJoint* const contact, dFloat timestep, const NewtonBody* const body0, const void* const collisionNode0, const NewtonBody* const body1, const void* const collisionNode1, int threadIndex)
  //  {
  //      //URHO3D_PROFILE_THREAD(NewtonThreadProfilerString(threadIndex).c_str());
  //      URHO3D_PROFILE_FUNCTION();

  //      return 1;
  //  }



  //  int Newton_WakeBodiesInAABBCallback(const NewtonBody* const body, void* const userData)
  //  {
  //      URHO3D_PROFILE_FUNCTION();
  //      //NewtonBodySetAutoSleep(body, 0);
  //      NewtonBodySetSleepState(body, 0);//wake the body.
  //      NewtonBodySetFreezeState(body, 0);
  //      return 1;
  //  }



}
