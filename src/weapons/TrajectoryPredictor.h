#pragma once
// Fırlatma yörüngesi öngörüsü: el bombası/fırlatılabilir için deterministik
// yörünge simülasyonu. Oyun tarafı dönen nokta listesiyle nişan çizgisi
// çizer; isteğe bağlı raycast ilk çarpma noktasında yörüngeyi keser (çarpma
// noktası ExplosionSystem::Detonate'e doğrudan verilebilir).
//
// Determinizm: sabit adımlı entegrasyon, RNG yok — aynı girdi her zaman
// aynı yörüngeyi verir (nişan çizgisi ile gerçek atış birebir örtüşür,
// yeter ki gerçek atış da aynı parametrelerle simüle edilsin).

#include "core/JoltAdapter.h"

#include <weapons/export.h>

#include <Jolt/Jolt.h>

#include <Jolt/Physics/PhysicsSystem.h>

#include <vector>

namespace alazforge {

struct TrajectoryConfig {
    float gravity = 9.81f;
    float dragFactor = 0.005f;     // 1/m kuadratik sürükleme (el bombası ~yavaş)
    Vec3 wind{0, 0, 0};            // WindSystem::GetWindAt sonucu verilebilir
    float timeStep = 1.0f / 60.0f; // örnekleme adımı (nokta yoğunluğu)
    float maxTime = 5.0f;          // s
};

struct TrajectoryResult {
    std::vector<Vec3> points; // origin dahil örneklenen yörünge noktaları
    bool hitSomething = false;
    Vec3 hitPoint{0, 0, 0};
    Vec3 hitNormal{0, 1, 0};
    JPH::BodyID hitBody;
    float flightTime = 0.0f;
};

class WEAPONS_API TrajectoryPredictor {
public:
    // physics nullptr olabilir: o zaman çarpma testi yapılmaz, yalnızca
    // serbest yörünge örneklenir (UI nişan çizgisi için yeterli).
    static TrajectoryResult Predict(JPH::PhysicsSystem* physics, const Vec3& origin,
                                    const Vec3& initialVelocity, const TrajectoryConfig& config);
};

} // namespace alazforge
