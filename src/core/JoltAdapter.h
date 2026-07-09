#pragma once
// AlazEngine (alazforge::Vec3/Quat/Transform) <-> Jolt (JPH::*) iki yönlü
// dönüşüm katmanı. Jolt double-precision (JPH_DOUBLE_PRECISION) ile
// derlenirse RVec3 dönüşümleri de doğru çalışır.

#include "core/AlazMath.h"

#include <Jolt/Jolt.h>
#include <Jolt/Math/Quat.h>
#include <Jolt/Math/Real.h>
#include <Jolt/Math/Vec3.h>

namespace alazforge {

// ── Vec3 ────────────────────────────────────────────────────────────
inline JPH::Vec3 ToJolt(const Vec3& v) { return JPH::Vec3(v.x, v.y, v.z); }

inline Vec3 FromJolt(const JPH::Vec3& v) { return Vec3{v.GetX(), v.GetY(), v.GetZ()}; }

// Jolt'un dünya pozisyonları RVec3. Tek hassasiyette RVec3 == Vec3 olduğu
// için ayrı overload yalnızca JPH_DOUBLE_PRECISION açıkken tanımlanır.
inline JPH::RVec3 ToJoltR(const Vec3& v) {
    return JPH::RVec3(static_cast<JPH::Real>(v.x), static_cast<JPH::Real>(v.y),
                      static_cast<JPH::Real>(v.z));
}

#ifdef JPH_DOUBLE_PRECISION
inline Vec3 FromJolt(const JPH::RVec3& v) {
    return Vec3{static_cast<f32>(v.GetX()), static_cast<f32>(v.GetY()), static_cast<f32>(v.GetZ())};
}
#endif

// ── Quat ────────────────────────────────────────────────────────────
inline JPH::Quat ToJolt(const Quat& q) { return JPH::Quat(q.x, q.y, q.z, q.w); }

inline Quat FromJolt(const JPH::Quat& q) { return Quat{q.GetX(), q.GetY(), q.GetZ(), q.GetW()}; }

// ── Transform ───────────────────────────────────────────────────────
// Not: Jolt rigid body'lerde scale taşımaz; scale AlazEngine tarafında
// kalır, buradan yalnızca position+rotation aktarılır.
inline Transform FromJolt(const JPH::RVec3& position, const JPH::Quat& rotation) {
    Transform t;
    t.position = FromJolt(position);
    t.rotation = FromJolt(rotation);
    return t;
}

} // namespace alazforge
