#pragma once
// Genel spor topu fizik sistemi: sürükleme (aerodinamik drag) + Magnus
// etkisi (spin'e bağlı kaldırma kuvveti — frikik/faul atışı eğrisi,
// basketbol backspin şutu, Amerikan futbolu spiral pasının eğilmesi).
//
// Bilinçli olarak sporlara özel hazır preset YOK — her topun kütlesi,
// yarıçapı, sürtünmesi, sekme katsayısı oyunu yapan tarafından
// BallConfig üzerinden belirlenir (futbol/basketbol/hentbol topu küre;
// Amerikan futbolu topu elongation>1 ile prolate spheroid/elips şekilde
// kurulabilir).
//
// Fizik modeli (oyun amaçlı basitleştirme, gerçek aerodinamik CFD değil):
//   F_surukleme = -0.5 * airDensity * dragCoefficient * A * |v| * v
//   F_magnus    = magnusCoefficient * airDensity * A * radius * (w x v)
// A = pi * radius^2 (ekvator kesiti; elongation'dan bağımsız — basitleştirme).
// magnusCoefficient boyutsuz bir ayar çarpanıdır (gerçek Magnus katsayısı
// topun yüzey pürüzlülüğüne/spin oranına bağlıdır, tek sabitle modellenmez);
// spin efekti istenmiyorsa 0 verilip kapatılabilir.

#include "core/JoltAdapter.h"

#include <sportsball/export.h>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/PhysicsSystem.h>

#include <vector>

namespace alazforge {

struct BallConfig {
    float mass = 0.45f;            // kg — oyunu yapan sporun topuna göre ayarlar
    float radius = 0.11f;          // m — ekvator/küre yarıçapı
    float elongation = 1.0f;       // >1 = prolate spheroid (örn. Amerikan futbolu
                                   // topu); 1.0 = tam küre (futbol/basketbol/hentbol)
    float dragCoefficient = 0.47f; // boyutsuz (pürüzsüz küre için tipik ~0.47)
    float restitution = 0.6f;
    float friction = 0.3f;
    float magnusCoefficient = 1.0f; // 0 = spin etkisi kapalı
    float airDensity = 1.225f;      // kg/m^3, deniz seviyesi varsayılan
};

class SPORTSBALL_API BallPhysicsSystem {
public:
    // Topun gövdesini oluşturur, ekler ve otomatik olarak izlemeye alır
    // (Update her frame sürükleme+Magnus kuvvetini uygular). elongation>1
    // ise JPH::ScaledShape ile küre yerelinin +z ekseninde uzatılmış
    // (prolate spheroid) bir şekil kurulur.
    JPH::BodyID CreateBall(JPH::PhysicsSystem& physics, const BallConfig& config,
                           const Vec3& worldPosition, const Quat& worldRotation,
                           JPH::ObjectLayer layer);

    void UntrackBall(JPH::BodyID body);

    // Her izlenen top için sürükleme + Magnus kuvvetini o topun mevcut
    // lineer/açısal hızına göre hesaplayıp uygular. Her frame çağrılması
    // beklenir (BuoyancySystem::Update ile aynı kullanım deseni).
    void Update(JPH::PhysicsSystem& physics, float dt);

    void RemoveBall(JPH::PhysicsSystem& physics, JPH::BodyID body);

private:
    struct TrackedBall {
        JPH::BodyID body;
        BallConfig config;
    };

    std::vector<TrackedBall> balls;
};

} // namespace alazforge
