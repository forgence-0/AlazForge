#include "wind/WindSystem.h"

#include <cmath>

namespace alazforge {

namespace {

constexpr float kTwoPi = 6.28318530717958647692f;

void Normalize(Vec3& v) {
    const float len = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    if (len > 1.0e-6f) {
        v.x /= len;
        v.y /= len;
        v.z /= len;
    } else {
        v = Vec3{1, 0, 0};
    }
}

} // namespace

WindSystem::WindSystem() : WindSystem(WindConfig{}) {}

WindSystem::WindSystem(const WindConfig& inConfig) { SetConfig(inConfig); }

void WindSystem::SetConfig(const WindConfig& inConfig) {
    config = inConfig;
    normalizedDir = config.direction;
    Normalize(normalizedDir);
}

Vec3 WindSystem::GetWindAt(const Vec3& worldPos) const {
    // Farkli frekansli uc sinusun toplami: periyodik gorunmeyen ama tamamen
    // deterministik bir "ruzgar esintisi" deseni. Pozisyon bileseni, genis
    // alanlarda ruzgarin ayni anda her yerde ayni olmamasini saglar.
    const float phase = config.spatialScale * (worldPos.x + 0.7f * worldPos.z);
    const float t = time * config.gustFrequency;
    const float gust =
        config.gustAmplitude * (0.55f * std::sin(kTwoPi * (t + phase)) +
                                0.30f * std::sin(kTwoPi * (2.3f * t + 1.7f * phase + 0.35f)) +
                                0.15f * std::sin(kTwoPi * (4.1f * t + 3.1f * phase + 0.71f)));

    const float speed = config.baseSpeed + gust;
    // Hafif yanal salinim: esinti yalnizca siddeti degil yonu de oynatir
    const float lateral =
        0.15f * config.gustAmplitude * std::sin(kTwoPi * (1.3f * t + 2.3f * phase + 0.13f));

    // normalizedDir'e dik yatay eksen (y-up varsayimi)
    const Vec3 side{-normalizedDir.z, 0.0f, normalizedDir.x};
    return Vec3{normalizedDir.x * speed + side.x * lateral, normalizedDir.y * speed,
                normalizedDir.z * speed + side.z * lateral};
}

Vec3 WindSystem::ComputeDragForce(const Vec3& worldPos, const Vec3& bodyVelocity,
                                  float dragCoefficient) const {
    const Vec3 wind = GetWindAt(worldPos);
    return Vec3{(wind.x - bodyVelocity.x) * dragCoefficient,
                (wind.y - bodyVelocity.y) * dragCoefficient,
                (wind.z - bodyVelocity.z) * dragCoefficient};
}

} // namespace alazforge
