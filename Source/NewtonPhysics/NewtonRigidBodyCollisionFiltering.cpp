
#include "NewtonPhysicsWorld.h"
#include "NewtonRigidBody.h"


#include "Urho3D/IO/Log.h"


namespace Urho3D
{

    void NewtonRigidBody::SetCollisionLayer(unsigned layer)
    {
        if (layer >= sizeof(unsigned) * 8)
            URHO3D_LOGWARNING("Collision Layer To Large");

        collisionLayer_ = layer;
    }

    void NewtonRigidBody::SetCollisionLayerMask(unsigned mask)
    {
        collisionLayerMask_ = mask;
    }

    void NewtonRigidBody::SetCollidableLayer(unsigned layer)
    {
        if (layer >= sizeof(unsigned) * 8)
            URHO3D_LOGWARNING("Collision Layer To Large");

        collisionLayerMask_ |= CollisionLayerAsBit(layer);
    }

    void NewtonRigidBody::UnSetCollidableLayer(unsigned layer)
    {
        if (layer >= sizeof(unsigned) * 8)
            URHO3D_LOGWARNING("Collision Layer To Large");
        collisionLayerMask_ &= ~CollisionLayerAsBit(layer);
    }

    void NewtonRigidBody::SetCollisionOverride(NewtonRigidBody* otherBody, bool enableCollisions)
    {
        if (otherBody == this)
            return;
        SetCollisionOverride(otherBody->GetID(), enableCollisions);
    }

    void NewtonRigidBody::SetCollisionOverride(unsigned otherBodyId, bool enableCollisions)
    {
        if (otherBodyId == this->GetID())
            return;

        if (collisionExceptions_.contains(StringHash(otherBodyId)))
        {
            collisionExceptions_[StringHash(otherBodyId)] = enableCollisions;
        }
        else
        {
            collisionExceptions_.insert_or_assign(StringHash(otherBodyId), Variant( enableCollisions ));
        }
    }

    void NewtonRigidBody::RemoveCollisionOverride(NewtonRigidBody* otherBody)
    {
        RemoveCollisionOverride(otherBody->GetID());
    }

    void NewtonRigidBody::RemoveCollisionOverride(unsigned otherBodyId)
    {
        collisionExceptions_.erase(StringHash(otherBodyId));
    }

	void NewtonRigidBody::GetCollisionExceptions(ea::vector<RigidBodyCollisionExceptionEntry>& exceptions)
    {
        for (auto pair : collisionExceptions_)
        {
            RigidBodyCollisionExceptionEntry entry;
            entry.enableCollisions_ = pair.second.GetBool();
            entry.rigidBodyComponentId_ = pair.first.Value();
            exceptions.push_front(entry);
        }
    }
    void NewtonRigidBody::SetNoCollideOverride(bool noCollide)
    {
        noCollideOverride_ = noCollide;
    }

    bool NewtonRigidBody::CanCollideWith(NewtonRigidBody* otherBody)
    {
        bool canCollide = true;


        //first check excpections because they have highest priority.
        bool exceptionsSpecified = false;
        if (noCollideOverride_)
        {
            canCollide = false;
            exceptionsSpecified = true;
        }
        if (otherBody->noCollideOverride_)
        {
            canCollide = false;
            exceptionsSpecified = true;
        }
        if (collisionExceptions_.contains(StringHash(otherBody->GetID())))
        {
            canCollide &= collisionExceptions_[StringHash(otherBody->GetID())].GetBool();
            exceptionsSpecified = true;
        }
        if (otherBody->collisionExceptions_.contains(StringHash(GetID())))
        {
            canCollide &= otherBody->collisionExceptions_[StringHash(GetID())].GetBool();
            exceptionsSpecified = true;
        }



        if (exceptionsSpecified)
            return canCollide;


        //now check collision masks/layers
        canCollide &= bool(collisionLayerMask_ & CollisionLayerAsBit(otherBody->collisionLayer_));
        canCollide &= bool(otherBody->collisionLayerMask_ & CollisionLayerAsBit(collisionLayer_));

        return canCollide;

    }



}

