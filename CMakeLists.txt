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
target_compile_definitions(rbfx-newton PRIVATE URHONEWTON_EXPORTS )
target_include_directories(rbfx-newton PUBLIC 
Source/NewtonPhysics
)

target_include_directories( rbfx-newton PUBLIC    ThirdParty/newton-dynamics/newton-4.00/sdk/dCore)
target_include_directories( rbfx-newton PUBLIC    ThirdParty/newton-dynamics/newton-4.00/sdk/dNewton)
target_include_directories( rbfx-newton PUBLIC    ThirdParty/newton-dynamics/newton-4.00/sdk/dNewton/dParticles)
target_include_directories( rbfx-newton PUBLIC    ThirdParty/newton-dynamics/newton-4.00/sdk/dNewton/dJoints)
target_include_directories( rbfx-newton PUBLIC    ThirdParty/newton-dynamics/newton-4.00/sdk/dNewton/dModels)
target_include_directories( rbfx-newton PUBLIC    ThirdParty/newton-dynamics/newton-4.00/sdk/dNewton/dModels/dVehicle)
target_include_directories( rbfx-newton PUBLIC    ThirdParty/newton-dynamics/newton-4.00/sdk/dNewton/dModels/dCharacter)
target_include_directories( rbfx-newton PUBLIC    ThirdParty/newton-dynamics/newton-4.00/sdk/dTinyxml)
target_include_directories( rbfx-newton PUBLIC    ThirdParty/newton-dynamics/newton-4.00/sdk/dCollision)


target_link_libraries(rbfx-newton PUBLIC 
    ndNewton
)

#newton plugins
#if(NEWTON_WITH_AVX_PLUGIN)
#	target_link_libraries(rbfx-newton PUBLIC 
#	    ndSolverAvx
#	)
#endif()
#if(NEWTON_WITH_AVX2_PLUGIN)
#	target_link_libraries(rbfx-newton PUBLIC 
#	    ndSolverAvx
#	)
#endif()


if(NEWTON_BUILD_SHARED_LIBS)
    add_definitions(-D_D_CORE_DLL)
    add_definitions(-D_D_TINY_DLL)
    add_definitions(-D_D_NEWTON_DLL)
    add_definitions(-D_D_COLLISION_DLL)
endif(NEWTON_BUILD_SHARED_LIBS)


#copy application media if sandbox is included
if(NEWTON_BUILD_SANDBOX_DEMOS)
    if(MSVC)
        #All 3 visual studio configurations:
        file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/bin/Debug/applications)
        file(COPY ThirdParty/newton-dynamics/newton-4.00/applications/media DESTINATION ${CMAKE_BINARY_DIR}/bin/Debug/applications)

        file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/bin/RelWithDebInfo/applications)
        file(COPY ThirdParty/newton-dynamics/newton-4.00/applications/media DESTINATION ${CMAKE_BINARY_DIR}/bin/RelWithDebInfo/applications)

        file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/bin/Release/applications)
        file(COPY ThirdParty/newton-dynamics/newton-4.00/applications/media DESTINATION ${CMAKE_BINARY_DIR}/bin/Release/applications)
    endif()
endif()



target_link_libraries(rbfx-newton PUBLIC Urho3D)


