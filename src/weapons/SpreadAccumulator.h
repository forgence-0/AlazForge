#pragma once
// Sekme/dağılım (spread) modellemesi (Faz B.2).
//
// BallisticsSystem'in "deterministik, rastgelelik yok" sözleşmesini korur:
// AlazForge kendi RNG'sini üretmez. SpreadAccumulator yalnızca ısı biriktirir/
// soğutur ve deterministik bir CurrentHalfAngleRad() döner; jitter'ı çağıran
// kendi RNG'siyle uygulamalıdır (ApplyConeJitter saf bir fonksiyondur, aynı
// (dir, halfAngle, u1, u2) her zaman aynı sonucu verir).
//
// Kasıtlı olarak Jolt'a bağımlı değil — yalnızca AlazMath POD tipleri ve
// <cmath> kullanır, silah mantığı fizik motorundan bağımsız test edilebilir.

#include "core/AlazMath.h"

#include <algorithm>
#include <cmath>

namespace alazforge {

struct SpreadConfig {
    float baseHalfAngleRad = 0.0017f; // ~0.1 derece, silahın mekanik hassasiyeti
    float heatPerShot = 0.15f;
    float maxHeat = 1.0f;
    float coolDownPerSecond = 0.3f;
    float maxHalfAngleRad = 0.05f; // tam ısınmış koni (~2.9 derece)
};

class SpreadAccumulator {
public:
    explicit SpreadAccumulator(const SpreadConfig& inConfig = SpreadConfig{}) : config(inConfig) {}

    // Isıyı ekler (maxHeat'e kenetli).
    void RegisterShot() { heat = std::min(heat + config.heatPerShot, config.maxHeat); }

    // Zamanla soğutur (0'a kenetli).
    void Update(float dt) { heat = std::max(heat - config.coolDownPerSecond * dt, 0.0f); }

    // heat -> lineer interpolasyon, tamamen deterministik (yalnızca iç duruma bağlı).
    float CurrentHalfAngleRad() const {
        const float t = config.maxHeat > 0.0f ? heat / config.maxHeat : 0.0f;
        return config.baseHalfAngleRad + t * (config.maxHalfAngleRad - config.baseHalfAngleRad);
    }

    float CurrentHeat() const { return heat; }

    void Reset() { heat = 0.0f; }

private:
    SpreadConfig config;
    float heat = 0.0f;
};

namespace detail {

inline float Length(const Vec3& v) { return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z); }

inline Vec3 Normalized(const Vec3& v) {
    const float len = Length(v);
    return len > 1e-8f ? Vec3{v.x / len, v.y / len, v.z / len} : Vec3{0, 0, 1};
}

inline Vec3 Cross(const Vec3& a, const Vec3& b) {
    return Vec3{a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

inline Vec3 Add(const Vec3& a, const Vec3& b, const Vec3& c) {
    return Vec3{a.x + b.x + c.x, a.y + b.y + c.y, a.z + b.z + c.z};
}

inline Vec3 Scale(const Vec3& v, float s) { return Vec3{v.x * s, v.y * s, v.z * s}; }

} // namespace detail

// Saf yardımcı — RNG'yi ÇAĞIRAN sağlar (u1,u2 in [0,1)). AlazForge kendi
// rastgelesini üretmez, bu fonksiyon deterministiktir: aynı girdi aynı çıktı.
// u1 koni içindeki yarıçap oranını, u2 açısal yönü belirler (disk örneklemesi).
inline Vec3 ApplyConeJitter(const Vec3& dir, float halfAngleRad, float u1, float u2) {
    const Vec3 forward = detail::Normalized(dir);

    // forward'a dik iki eksen bul
    const Vec3 arbitrary = std::fabs(forward.y) < 0.99f ? Vec3{0, 1, 0} : Vec3{1, 0, 0};
    const Vec3 right = detail::Normalized(detail::Cross(forward, arbitrary));
    const Vec3 up = detail::Cross(right, forward);

    const float radius = std::sqrt(u1) * std::sin(halfAngleRad);
    const float theta = 2.0f * 3.14159265f * u2;
    const float x = radius * std::cos(theta);
    const float y = radius * std::sin(theta);
    const float z = std::sqrt(std::max(0.0f, 1.0f - radius * radius));

    const Vec3 jittered =
        detail::Add(detail::Scale(right, x), detail::Scale(up, y), detail::Scale(forward, z));
    return detail::Normalized(jittered);
}

} // namespace alazforge
