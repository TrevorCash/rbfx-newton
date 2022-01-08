#
# Copyright (c) 2017-2019 Rokas Kupstys.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#

# Set CMake minimum version and CMake policy required by UrhoCommon module
cmake_minimum_required (VERSION 3.4.3)

# Set project name
project (rbfx-newton)

# Dependencies
add_subdirectory(ThirdParty/newton-dynamics/newton-4.00)

# Library
file(GLOB all_SRCS
        "${PROJECT_SOURCE_DIR}/Source/NewtonPhysics/*.h"
        "${PROJECT_SOURCE_DIR}/Source/NewtonPhysics/*.cpp"
        )

add_library(rbfx-newton ${all_SRCS})
target_compile_definitions(rbfx-newton PRIVATE URHONEWTON_EXPORTS)
target_include_directories(rbfx-newton PUBLIC 
Source/NewtonPhysics
)

include_directories(ThirdParty/newton-dynamics/newton-4.00/sdk/dCore)
include_directories(ThirdParty/newton-dynamics/newton-4.00/sdk/dNewton)
include_directories(ThirdParty/newton-dynamics/newton-4.00/sdk/dNewton/dJoints)
include_directories(ThirdParty/newton-dynamics/newton-4.00/sdk/dNewton/dModels)
include_directories(ThirdParty/newton-dynamics/newton-4.00/sdk/dNewton/dModels/dVehicle)
include_directories(ThirdParty/newton-dynamics/newton-4.00/sdk/dNewton/dModels/dCharacter)
include_directories(ThirdParty/newton-dynamics/newton-4.00/sdk/dTinyxml)
include_directories(ThirdParty/newton-dynamics/newton-4.00/sdk/dCollision)


target_link_libraries(rbfx-newton PUBLIC 
    dAnimation
    dContainers
    dCustomJoints
    dgCore
    dgPhysics
    dNewton
    dMath
    newton
)

#newton plugins
if(NEWTON_WITH_AVX_PLUGIN)
	target_link_libraries(rbfx-newton PUBLIC 
	    dgNewtonAvx
	)
endif()
if(NEWTON_WITH_AVX2_PLUGIN)
	target_link_libraries(rbfx-newton PUBLIC 
	    dgNewtonAvx2
	)
endif()
if(NEWTON_WITH_SSE4_PLUGIN)
	target_link_libraries(rbfx-newton PUBLIC 
	    dgNewtonSse4
	)
endif()








include(CheckIncludeFile)
check_include_file("valgrind/valgrind.h" HAVE_VALGRIND)
if (HAVE_VALGRIND)
    target_compile_definitions(rbfx-newton PRIVATE -DHAVE_VALGRIND=${HAVE_VALGRIND})
endif ()


target_link_libraries(rbfx-newton PUBLIC Urho3D)



if (URHO3D_SAMPLES)
	add_subdirectory(Samples/NewtonPhysicsTests)
endif ()

