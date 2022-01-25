#include "UrhoNewtonConversions.h"


#include "Urho3D/Math/Matrix4.h"
#include "Urho3D/Math/Matrix3x4.h"
#include "Urho3D/Math/Sphere.h"
#include "Urho3D/Math/BoundingBox.h"
#include "Urho3D/IO/Log.h"


#include "ndNewton.h"


namespace Urho3D {


    ndMatrix UrhoToNewton(const Matrix4& mat4)
    {
#ifndef _NEWTON_USE_DOUBLE
        return ndMatrix(mat4.Transpose().Data());
#else

        Matrix4 tranposed = mat4.Transpose();
        const float* dataPtr = tranposed.Data();
        dFloat data[16];

        for (int i = 0; i < 16; i++)
            data[i] = dataPtr[i];


        return dMatrix(data);

#endif
    }
    ndMatrix UrhoToNewton(const Matrix3x4& mat3x4)
    {
#ifndef _NEWTON_USE_DOUBLE
        Matrix4 asMat4 = mat3x4.ToMatrix4();
        return ndMatrix(asMat4.Transpose().Data());
#else
        Matrix4 tranposed = mat3x4.ToMatrix4().Transpose();
        const float* dataPtr = tranposed.Data();
        dFloat data[16];

        for (int i = 0; i < 16; i++)
            data[i] = dataPtr[i];


        return ndMatrix(data);
#endif
    }
	ndMatrix UrhoToNewton(const Matrix3& mat3)
	{
#ifndef _NEWTON_USE_DOUBLE
		Matrix4 asMat4 = Matrix4(mat3);
		return ndMatrix(asMat4.Transpose().Data());
#else
		Matrix4 tranposed = Matrix4(mat3).Transpose();
		const float* dataPtr = tranposed.Data();
		dFloat data[16];

		for (int i = 0; i < 16; i++)
			data[i] = dataPtr[i];


		return ndMatrix(data);
#endif
	}

    ndVector UrhoToNewton(const Vector4& vec4)
    {
        return {vec4.x_, vec4.y_, vec4.z_, vec4.w_};
    }

    ndVector UrhoToNewton(const Vector3& vec3)
    {
        return ndVector(vec3.x_, vec3.y_, vec3.z_, 0.0f);
    }

    ndVector UrhoToNewton(const Vector2& vec2)
    {
        return ndVector(vec2.x_, vec2.y_, 0.0f, 0.0f);
    }
    ndQuaternion UrhoToNewton(const Quaternion& quat)
    {
        return {quat.w_, quat.x_, quat.y_, quat.z_};
    }




    Vector3 NewtonToUrhoVec3(const ndVector& vec)
    {
        return {vec.m_x, vec.m_y, vec.m_z};
    }
    Vector4 NewtonToUrhoVec4(const ndVector& vec)
    {
        return {vec.m_x, vec.m_y, vec.m_z, vec.m_w};
    }


    Matrix4 NewtonToUrhoMat4(const ndMatrix& mat)
    {
#ifndef _NEWTON_USE_DOUBLE
        return Matrix4(&mat[0][0]).Transpose();
#else
        float data[16];
        for (int r = 0; r < 4; r++) {
            for (int c = 0; c < 4; c++) {
                data[r + 4*c] = mat[c][r];
            }
        }

        return Matrix4(data).Transpose();


#endif
    }

    Quaternion NewtonToUrhoQuat(const ndQuaternion& quat)
    {
        return {quat.m_w, quat.m_x, quat.m_y, quat.m_z};
    }
    // Quaternion NewtonToUrhoQuat(const dgQuaternion& quat)
    // {
    //     return  Quaternion(quat.m_w, quat.m_x, quat.m_y, quat.m_z);
    // }


    ndShape* UrhoShapeToNewtonCollision(const NewtonWorld* newtonWorld, const Sphere& sphere)
    {
        Matrix3x4 mat;
        mat.SetTranslation(sphere.center_);
        ndMatrix dMat = UrhoToNewton(mat);

        ndShape* newtonShape = new ndShapeSphere(sphere.radius_);
 
        return newtonShape;
    }


    ndShape* UrhoShapeToNewtonCollision(const NewtonWorld* newtonWorld, const BoundingBox& box)
    {
        Matrix3x4 mat;
        mat.SetTranslation(box.Center());
        ndMatrix dMat = UrhoToNewton(mat);

        ndShape* newtonShape = new ndShapeBox(box.Size().x_, box.Size().y_, box.Size().z_);

        return newtonShape;
    }


    void PrintNewtonMatrix(ndMatrix mat)
    {
        const int paddingSize = 10;
        for (int row = 0; row < 4; row++) {

            ea::vector<ea::string> rowString;

            for (int col = 0; col < 4; col++) {
                eastl::string colString = eastl::to_string(mat[row][col]);

				while (colString.length() < paddingSize)
					colString.push_back(' ');



				rowString.push_back(colString);

            }


            URHO3D_LOGINFO(eastl::string(rowString[0]) + " , " + eastl::string(rowString[1]) + " , " + eastl::string(rowString[2]) + " , " + eastl::string(rowString[3]));
        }


        URHO3D_LOGINFO("");

    }



	

}
