cmake_minimum_required(VERSION 3.29)
cmake_policy(SET CMP0077 NEW)
set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)

project(tacoEngine)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS -fno-rtti)

set(RAPIDJSON_BUILD_EXAMPLES OFF)
set(RAPIDJSON_BUILD_TESTS OFF)

add_subdirectory(ext/entt)
add_subdirectory(ext/JoltPhysics/Build)
add_subdirectory(ext/rapidjson)
add_subdirectory(ext/tacoRender)
add_subdirectory(ext/yal)

set(TACO_SOURCES
        src/taco/comp/Camera.h
        src/taco/comp/Lights.cpp
        src/taco/comp/Lights.h
        src/taco/comp/Transform.h
        src/taco/misc/Debug.cpp
        src/taco/misc/Debug.h
        src/taco/misc/Log.cpp
        src/taco/misc/Log.h
        src/taco/Config.h
        src/taco/Engine.cpp
        src/taco/Engine.h
        src/taco/Graphics.h
        src/taco/Physics.cpp
        src/taco/Physics.h
        src/taco/misc/Rotation.cpp
        src/taco/misc/Rotation.h
)

add_library(tacoEngine ${TACO_SOURCES})

target_link_libraries(tacoEngine EnTT::EnTT tacoRender yal Jolt)
target_include_directories(tacoEngine PUBLIC src)
