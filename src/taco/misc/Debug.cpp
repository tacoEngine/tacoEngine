// game (c) Nikolas Wipper 2024

#include "Debug.h"

#include <raylib.h>

Vector3 torl(JPH::RVec3Arg vec) {
    return {vec.GetX(), vec.GetY(), vec.GetZ()};
}

Vector4 torl(JPH::Vec4 vec) {
    return {vec.GetX(), vec.GetY(), vec.GetZ(), vec.GetW()};
}

Color torl(JPH::ColorArg color) {
    return {color.r, color.g, color.b, color.a};
}

Matrix torl(JPH::RMat44Arg mat) {
    Vector4 col0 = torl(mat.GetColumn4(0));
    Vector4 col1 = torl(mat.GetColumn4(1));
    Vector4 col2 = torl(mat.GetColumn4(2));
    Vector4 col3 = torl(mat.GetColumn4(3));
    return {
        col0.x,
        col1.x,
        col2.x,
        col3.x,
        col0.y,
        col1.y,
        col2.y,
        col3.y,
        col0.z,
        col1.z,
        col2.z,
        col3.z,
        col0.w,
        col1.w,
        col2.w,
        col3.w,
    };
}

void taco::RaylibDebugRenderer::DrawLine(JPH::RVec3Arg inFrom, JPH::RVec3Arg inTo, JPH::ColorArg inColor) {
    DrawLine3D(torl(inFrom), torl(inTo), torl(inColor));
}

void taco::RaylibDebugRenderer::DrawTriangle(JPH::RVec3Arg inV1, JPH::RVec3Arg inV2, JPH::RVec3Arg inV3,
                                       JPH::ColorArg inColor, ECastShadow inCastShadow) {
    DrawTriangle3D(torl(inV1), torl(inV2), torl(inV3), torl(inColor));
}

void taco::RaylibDebugRenderer::DrawText3D(JPH::RVec3Arg inPosition, const std::string_view &inString, JPH::ColorArg inColor,
                                     float inHeight) {}
