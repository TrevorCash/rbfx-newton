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

#include "NewtonCollisionShape.h"
#include "NewtonPhysicsWorld.h"
#include "NewtonRigidBody.h"
#include "NewtonMeshObject.h"
#include "UrhoNewtonConversions.h"
#include "NewtonDebugDrawing.h"

#include "Urho3D/Core/Context.h"
#include "Urho3D/Scene/Component.h"
#include "Urho3D/Scene/Node.h"
#include "Urho3D/Scene/Scene.h"
#include "Urho3D/Graphics/Model.h"
#include "Urho3D/IO/Log.h"
#include "Urho3D/Graphics/Geometry.h"
#include "Urho3D/Graphics/VertexBuffer.h"
#include "Urho3D/Graphics/GraphicsDefs.h"
#include "Urho3D/Graphics/IndexBuffer.h"
#include "Urho3D/Graphics/StaticModel.h"
#include "Urho3D/Scene/SceneEvents.h"




namespace Urho3D {



    NewtonCollisionShape::NewtonCollisionShape(Context* context) : Component(context),
	newtonShape_(nullptr)
    {
        SubscribeToEvent(E_NODEADDED, URHO3D_HANDLER(NewtonCollisionShape, HandleNodeAdded));
        SubscribeToEvent(E_NODEREMOVED, URHO3D_HANDLER(NewtonCollisionShape, HandleNodeRemoved));
    }

    NewtonCollisionShape::~NewtonCollisionShape()
    {
    }

    void NewtonCollisionShape::RegisterObject(Context* context)
    {
        context->RegisterFactory<NewtonCollisionShape>(DEF_PHYSICS_CATEGORY.c_str());
        URHO3D_COPY_BASE_ATTRIBUTES(Component);

        URHO3D_ACCESSOR_ATTRIBUTE("Position Offset", GetPositionOffset, SetPositionOffset, Vector3, Vector3::ZERO, AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("Rotational Offset", GetRotationOffset, SetRotationOffset, Quaternion, Quaternion::IDENTITY, AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("Scale Factor", GetScaleFactor, SetScaleFactor, Vector3, Vector3::ONE, AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("Inherit Collision Node Scales", GetInheritNodeScale, SetInheritNodeScale, bool, true, AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("Static Friction Coefficient", GetStaticFriction, SetStaticFriction, float, COLLISION_SHAPE_DEF_STATIC_FRICTION, AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("Kinetic Friction Coefficient", GetKineticFriction, SetKineticFriction, float, COLLISION_SHAPE_DEF_KINETIC_FRICTION, AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("Elasticity", GetElasticity, SetElasticity, float, COLLISION_SHAPE_DEF_ELASTICITY, AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("Softness", GetSoftness, SetSoftness, float, COLLISION_SHAPE_DEF_SOFTNESS, AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("Density", GetDensity, SetDensity, float, 1.0f, AM_DEFAULT);
    }




    void NewtonCollisionShape::SetFriction(float friction)
    {
        SetStaticFriction(friction);
        SetKineticFriction(friction - 0.1f);
    }

    void NewtonCollisionShape::updateBuild()
    {
            //call the derived class createNewtonCollision function.
            if (buildNewtonCollision())
            {


                ////determine if the built collision is a compound.
                //int numSubShapes = 0;
                //void* curSubColNode = NewtonCompoundCollisionGetFirstNode(newtonShape_);
                //while (curSubColNode) {
                //    numSubShapes++;

                //    //set user data on each part.
                //    NewtonCollisionSetUserData(NewtonCompoundCollisionGetCollisionFromNode(newtonShape_, curSubColNode), (void*)this);

                //    curSubColNode = NewtonCompoundCollisionGetNextNode(newtonShape_, curSubColNode);


                //}

                //if (numSubShapes > 1)
                //    isCompound_ = true;
                //else
                //    isCompound_ = false;
				

                //NewtonCollisionSetUserData(newtonShape_, (void*)this);
            }
    }

    bool NewtonCollisionShape::buildNewtonCollision()
    {
        return true;
    }





    Urho3D::Matrix3x4 NewtonCollisionShape::GetOffsetMatrix()
    {
        return Matrix3x4(position_, rotation_, scale_);
    }

    Urho3D::Quaternion NewtonCollisionShape::GetWorldRotation()
    {
        return node_->GetWorldRotation() * GetRotationOffset();
    }

    Urho3D::Vector3 NewtonCollisionShape::GetWorldPosition()
    {
        return GetWorldTransform().Translation();
    }

    Urho3D::Matrix3x4 NewtonCollisionShape::GetWorldTransform()
    {
        if (!node_)
        {
            URHO3D_LOGWARNING("CollisionShape::GetWorldTransform No Node Present");
            return Matrix3x4();

        }

        if(GetInheritNodeScale())
            return node_->GetWorldTransform() * GetOffsetMatrix();
        else
        {
            return Matrix3x4(node_->GetWorldTransform().Translation(), node_->GetWorldTransform().Rotation(), 1.0f) * GetOffsetMatrix();
        }
    }



    void NewtonCollisionShape::SetStaticFriction(float staticFriction)
    {
        if (staticFriction_ != staticFriction)
            staticFriction_ = staticFriction;
    }

    float NewtonCollisionShape::GetStaticFriction() const
    {
        return staticFriction_;
    }

    void NewtonCollisionShape::SetKineticFriction(float kineticFriction)
    {
        if (kineticFriction_ != kineticFriction)
        {
            kineticFriction_ = kineticFriction;
        }
    }

    float NewtonCollisionShape::GetKineticFriction() const
    {
        return kineticFriction_;
    }

    void NewtonCollisionShape::SetElasticity(float elasticity)
    {
        if (elasticity_ != elasticity)
        {
            elasticity_ = elasticity;
        }
    }

    float NewtonCollisionShape::GetElasticity() const
    {
        return elasticity_;
    }

    void NewtonCollisionShape::SetSoftness(float softness)
    {
        if (softness_ != softness)
        {
            softness_ = softness;
        }
    }

    float NewtonCollisionShape::GetSoftness() const
    {
        return softness_;
    }

    void NewtonCollisionShape::SetDensity(float density)
    {
        if (density != density_)
        {
            density_ = density;
            MarkDirty();
        }
    }

    void NewtonCollisionShape::SetPositionOffset(Vector3 position)
    {
        position_ = position; MarkDirty(true);
    }

    void NewtonCollisionShape::SetScaleFactor(float scale)
    {
        scale_ = Vector3(scale, scale, scale); MarkDirty(true);
    }

    void NewtonCollisionShape::SetScaleFactor(Vector3 scale)
    {
        scale_ = scale; MarkDirty(true);
    }

    void NewtonCollisionShape::SetRotationOffset(Quaternion rotation)
    {
        rotation_ = rotation; MarkDirty(true);
    }

    Urho3D::Vector3 NewtonCollisionShape::GetPositionOffset() const
    {
        return position_;
    }

    Urho3D::Vector3 NewtonCollisionShape::GetScaleFactor() const
    {
        return scale_;
    }

    Urho3D::Quaternion NewtonCollisionShape::GetRotationOffset() const
    {
        return rotation_;
    }

    void NewtonCollisionShape::SetInheritNodeScale(bool enable /*= true*/)
    {
        if (inheritCollisionNodeScales_ != enable) {
            inheritCollisionNodeScales_ = enable;
            MarkDirty();//we need to rebuild for this change
        }

    }



    bool NewtonCollisionShape::GetInheritNodeScale() const
    {
        return inheritCollisionNodeScales_;
    }

    void NewtonCollisionShape::MarkDirty(bool dirty /*= true*/)
    {
        if (shapeNeedsRebuilt_ != dirty) {
            shapeNeedsRebuilt_ = dirty;

            // always mark the rigid body dirty as well if the collision shape is dirty.
            if(dirty)
                MarkRigidBodyDirty();
        }
    }

    bool NewtonCollisionShape::GetDirty() const
    {
        return shapeNeedsRebuilt_;
    }

    bool NewtonCollisionShape::IsCompound() const
    {
        return isCompound_;
    }

    const ndShapeInstance NewtonCollisionShape::GetNewtonShape()
    {
        return newtonShape_;
    }


    bool NewtonCollisionShape::GetDrawNewtonDebugGeometry()
    {
        return drawPhysicsDebugCollisionGeometry_;
    }

    void NewtonCollisionShape::SetDrawNewtonDebugGeometry(bool enable)
    {
        drawPhysicsDebugCollisionGeometry_ = enable;
    }



    void NewtonCollisionShape::OnSetEnabled()
    {
        MarkRigidBodyDirty();
    }


    NewtonRigidBody* NewtonCollisionShape::GetRigidBody()
    {
		ea::vector<NewtonRigidBody*> rootRigidBodies;
        GetRootRigidBodies(rootRigidBodies, node_, true);
        if (rootRigidBodies.size() > 1)
            return rootRigidBodies[rootRigidBodies.size() - 2];
        else if (rootRigidBodies.size() == 1)
        {
            return rootRigidBodies.back();//return scene rigid body.
        }

        return nullptr;
    }

    void NewtonCollisionShape::MarkRigidBodyDirty()
    {
        GetRigidBody()->MarkDirty(true);
    }

    void NewtonCollisionShape::OnNodeSet(Node* node)
    {

        if (node)
        {
            ///auto create physics world
			physicsWorld_ = WeakPtr<NewtonPhysicsWorld>(GetScene()->GetOrCreateComponent<NewtonPhysicsWorld>());


            physicsWorld_->addCollisionShape(this);
            node->AddListener(this);
        }
        else
        {
            if (physicsWorld_) {
                physicsWorld_->WaitForUpdateFinished();

            }

        }

    }



    void NewtonCollisionShape::OnNodeSetEnabled(Node* node)
    {
        MarkRigidBodyDirty();
    }

    void NewtonCollisionShape::HandleNodeAdded(StringHash event, VariantMap& eventData)
    {
        Node* node = static_cast<Node*>(eventData[NodeAdded::P_NODE].GetPtr());
        Node* newParent = static_cast<Node*>(eventData[NodeRemoved::P_PARENT].GetPtr());

        if (node == node_)
        {
            RebuildPhysicsNodeTree(node);
        }
    }

    void NewtonCollisionShape::HandleNodeRemoved(StringHash event, VariantMap& eventData)
    {
        Node* node = static_cast<Node*>(eventData[NodeRemoved::P_NODE].GetPtr());
        if (node == node_)
        {
            Node* oldParent = static_cast<Node*>(eventData[NodeRemoved::P_PARENT].GetPtr());

            if (oldParent)
            {
                RebuildPhysicsNodeTree(oldParent);
            }
            else
            {
                URHO3D_LOGINFO("should not happen");
            }
        }
    }

}
