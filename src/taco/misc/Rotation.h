// game (c) Nikolas 2025

#ifndef ROTATION_H
#define ROTATION_H

#include <raymath.h>

namespace taco {
class Rotation : public Vector3 {
    mutable Quaternion cached_quat_ = QuaternionIdentity();
    mutable Vector3 cached_rot_ = Vector3Zero();
public:
    Rotation(float x = 0.f, float y = 0.f, float z = 0.f);

    Quaternion GetQuaternion() const;
    void SetFromQuaternion(Quaternion quaternion);
};
} // taco

#endif //ROTATION_H
