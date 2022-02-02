#pragma once
#include "UrhoNewtonApi.h"


class NewtonCollision;
class NewtonWorld;

class ndVector;
class ndShape;
class ndMatrix;
class ndQuaternion;

namespace Urho3D {
    class Matrix4;
    class Matrix3x4;
	class Matrix3;
    class Vector2;
    class Vector3;
    class Vector4;
    class Quaternion;
    class Sphere;
    class BoundingBox;

    ///Conversion Functions From Urho To Newton ndMatrix UrhoToNewton(const Matrix4& mat);
    URHONEWTON_API ndMatrix UrhoToNewton(const Matrix3x4& mat);
    URHONEWTON_API ndMatrix UrhoToNewton(const Matrix3& mat3);
    URHONEWTON_API ndVector UrhoToNewton(const Vector4& vec4);
    URHONEWTON_API ndVector UrhoToNewton(const Vector3& vec3);
    URHONEWTON_API ndVector UrhoToNewton(const Vector2& vec2);
    URHONEWTON_API ndQuaternion UrhoToNewton(const Quaternion& quat);

    ///Conversion Function From Newton To Urho
    URHONEWTON_API Vector3 NewtonToUrhoVec3(const ndVector& vec);
    URHONEWTON_API Vector4 NewtonToUrhoVec4(const ndVector& vec);
    URHONEWTON_API Matrix4 NewtonToUrhoMat4(const ndMatrix& mat);
    //Quaternion NewtonToUrhoQuat(const dQuaternion& quat);
    URHONEWTON_API Quaternion NewtonToUrhoQuat(const ndQuaternion& quat);



    ///shape conversion

    ///return a newton collision from an urho shape - optionally include the translation of the shape in the collision. Remember to NewtonDestroy the NewtonCollision when you are done with it!
    URHONEWTON_API ndShape* UrhoShapeToNewtonCollision(const NewtonWorld* newtonWorld, const Sphere& sphere);
    URHONEWTON_API ndShape* UrhoShapeToNewtonCollision(const NewtonWorld* newtonWorld, const BoundingBox& box);




	//void* NewtonCompoundCollisionGetFirstNode(ndShapeInstance* const compoundCollision);
	//void* NewtonCompoundCollisionGetNextNode(ndShapeInstance* const compoundCollision, const void* const node);
	//void* NewtonCompoundCollisionGetNodeByIndex(ndShapeInstance* const compoundCollision, int index);
	//int NewtonCompoundCollisionGetNodeIndex(ndShapeInstance* const compoundCollision, const void* const node);
	//ndShapeInstance* NewtonCompoundCollisionGetCollisionFromNode(ndShapeInstance* const compoundCollision, const void* const node);




    ///Printing Helpers
    URHONEWTON_API void PrintNewtonMatrix(ndMatrix mat);











}
