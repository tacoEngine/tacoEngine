cmake_minimum_required(VERSION 3.29)
project(tacoEngine)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS -fno-rtti)

add_subdirectory(ext/entt)
add_subdirectory(ext/JoltPhysics/Build)
add_subdirectory(ext/rapidjson)
add_subdirectory(ext/tacoRender)
add_subdirectory(ext/yal)

set(TACO_SOURCES
        src/taco/Engine.cpp
        src/taco/Engine.h
        src/taco/Camera.h
        src/taco/Transform.h
        src/taco/Graphics.h
        src/taco/Log.cpp
        src/taco/Log.h
        src/taco/Physics.h
        src/taco/Physics.cpp
        src/taco/Debug.cpp
        src/taco/Debug.h
        src/taco/Config.h
        src/taco/Lights.h
)

add_library(tacoEngine ${TACO_SOURCES})

target_link_libraries(tacoEngine EnTT::EnTT tacoRender yal Jolt)
target_include_directories(tacoEngine PUBLIC src)
