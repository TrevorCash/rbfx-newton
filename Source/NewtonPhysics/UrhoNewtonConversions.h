#pragma once
#include "ndShape.h"


class dMatrix;
class dVector;
class dQuaternion;
class dgQuaternion;

class NewtonCollision;
class NewtonWorld;

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

    ///Conversion Functions From Urho To Newton
    ndMatrix UrhoToNewton(const Matrix4& mat);
    ndMatrix UrhoToNewton(const Matrix3x4& mat);
	ndMatrix UrhoToNewton(const Matrix3& mat3);
    ndVector UrhoToNewton(const Vector3& vec4);
    ndVector UrhoToNewton(const Vector3& vec3);
    ndVector UrhoToNewton(const Vector2& vec2);
    ndQuaternion UrhoToNewton(const Quaternion& quat);

    ///Conversion Function From Newton To Urho
    Vector3 NewtonToUrhoVec3(const ndVector& vec);
    Vector4 NewtonToUrhoVec4(const ndVector& vec);
    Matrix4 NewtonToUrhoMat4(const ndMatrix& mat);
    //Quaternion NewtonToUrhoQuat(const dQuaternion& quat);
    Quaternion NewtonToUrhoQuat(const ndQuaternion& quat);



    ///shape conversion

    ///return a newton collision from an urho shape - optionally include the translation of the shape in the collision. Remember to NewtonDestroy the NewtonCollision when you are done with it!
    ndShape* UrhoShapeToNewtonCollision(const NewtonWorld* newtonWorld, const Sphere& sphere);
    ndShape* UrhoShapeToNewtonCollision(const NewtonWorld* newtonWorld, const BoundingBox& box);








    ///Printing Helpers
    void PrintNewtonMatrix(ndMatrix mat);











}
