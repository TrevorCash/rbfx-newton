#include "Urho3D/Scene/Component.h"
#include "UrhoNewtonApi.h"
class ndModel;

namespace Urho3D
{
	class NewtonConstraint;

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
    protected:
	    void OnSceneSet(Scene* scene) override;

        ea::vector<NewtonRigidBody*> bodies;
        ea::vector<NewtonConstraint*> constraints;

    	ndModel* newtonModel = nullptr;


        WeakPtr<NewtonPhysicsWorld> physicsWorld_ = nullptr;
    };


}
