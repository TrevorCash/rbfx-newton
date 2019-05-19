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

#include "Urho3D/Core/Object.h"

namespace Urho3D
{

/// Physics world is about to be stepped.
URHO3D_EVENT(E_NEWTON_PHYSICSPRESTEP, NewtonPhysicsPreStep)
{
    URHO3D_PARAM(P_WORLD, World);                  // PhysicsWorld pointer
    URHO3D_PARAM(P_TIMESTEP, TimeStep);            // float
}

/// Physics world has been stepped.
URHO3D_EVENT(E_NEWTON_PHYSICSPOSTSTEP, NewtonPhysicsPostStep)
{
    URHO3D_PARAM(P_WORLD, World);                  // PhysicsWorld pointer
    URHO3D_PARAM(P_TIMESTEP, TimeStep);            // float
}

/// Physics collision started. Global event sent by the PhysicsWorld.
URHO3D_EVENT(E_NEWTON_PHYSICSCOLLISIONSTART, NewtonPhysicsCollisionStart)
{
    URHO3D_PARAM(P_WORLD, World);                  // PhysicsWorld pointer
    URHO3D_PARAM(P_BODYA, BodyA);                  // RigidBody pointer
    URHO3D_PARAM(P_BODYB, BodyB);                  // RigidBody pointer
    URHO3D_PARAM(P_CONTACT_DATA, Contacts);        // NewtonRigidBodyContactEntry pointer containing information relating to contacts. 
}

/// Physics collision ongoing. Global event sent by the PhysicsWorld.
URHO3D_EVENT(E_NEWTON_PHYSICSCOLLISION, NewtonPhysicsCollision)
{
    URHO3D_PARAM(P_WORLD, World);                  // PhysicsWorld pointer
    URHO3D_PARAM(P_BODYA, BodyA);                  // RigidBody pointer
    URHO3D_PARAM(P_BODYB, BodyB);                  // RigidBody pointer
    URHO3D_PARAM(P_CONTACT_DATA, Contacts);        // NewtonRigidBodyContactEntry pointer containing information relating to contacts. 
}

/// Physics collision ended. Global event sent by the PhysicsWorld.
URHO3D_EVENT(E_NEWTON_PHYSICSCOLLISIONEND, NewtonPhysicsCollisionEnd)
{
    URHO3D_PARAM(P_WORLD, World);                  // PhysicsWorld pointer
    URHO3D_PARAM(P_BODYA, BodyA);                  // RigidBody pointer
    URHO3D_PARAM(P_BODYB, BodyB);                  // RigidBody pointer
}

/// Node's physics collision started. Sent by scene nodes participating in a collision.
URHO3D_EVENT(E_NEWTON_NODECOLLISIONSTART, NewtonNodeCollisionStart)
{
    URHO3D_PARAM(P_BODY, Body);                    // RigidBody pointer
    URHO3D_PARAM(P_OTHERNODE, OtherNode);          // Node pointer
    URHO3D_PARAM(P_OTHERBODY, OtherBody);          // RigidBody pointer
    URHO3D_PARAM(P_TRIGGER, Trigger);              // bool
    URHO3D_PARAM(P_CONTACT_DATA, Contacts);        // NewtonRigidBodyContactEntry pointer containing information relating to contacts. 
}

/// Node's physics collision ongoing. Sent by scene nodes participating in a collision.
URHO3D_EVENT(E_NEWTON_NODECOLLISION, NewtonNodeCollision)
{
    URHO3D_PARAM(P_BODY, Body);                    // RigidBody pointer
    URHO3D_PARAM(P_OTHERNODE, OtherNode);          // Node pointer
    URHO3D_PARAM(P_OTHERBODY, OtherBody);          // RigidBody pointer
    URHO3D_PARAM(P_TRIGGER, Trigger);              // bool
    URHO3D_PARAM(P_CONTACT_DATA, Contacts);        // NewtonRigidBodyContactEntry pointer containing information relating to contacts. 
}

/// Node's physics collision ended. Sent by scene nodes participating in a collision.
URHO3D_EVENT(E_NEWTON_NODECOLLISIONEND, NewtonNodeCollisionEnd)
{
    URHO3D_PARAM(P_BODY, Body);                    // RigidBody pointer
    URHO3D_PARAM(P_OTHERNODE, OtherNode);          // Node pointer
    URHO3D_PARAM(P_OTHERBODY, OtherBody);          // RigidBody pointer
    URHO3D_PARAM(P_TRIGGER, Trigger);              // bool
}

}
