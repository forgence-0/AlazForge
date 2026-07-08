#pragma once
// AlazEngine'in Physics/PhysicsEngine.hpp içindeki matematik tipleriyle
// birebir aynı bellek düzeni (POD). AlazForge, AlazEngine'e bağımlı olmadan
// derlenebilsin diye tipler burada aynalanır; entegrasyonda reinterpret
// gerekmeden doğrudan kopyalanabilir.

#include <cstdint>

namespace alazforge {

using f32 = float;

struct Vec3 {
    f32 x = 0, y = 0, z = 0;
};

struct Quat {
    f32 x = 0, y = 0, z = 0, w = 1;
};

struct Transform {
    Vec3 position;
    Quat rotation;
    Vec3 scale{1, 1, 1};
};

} // namespace alazforge
