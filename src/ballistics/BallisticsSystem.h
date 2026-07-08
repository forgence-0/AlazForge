#pragma once
// Raycasting tabanlı balistik sistem (Faz 4).
//
// Mermi, küçük zaman adımlarıyla simüle edilir: her adımda yerçekimi düşüşü
// ve hava direnci uygulanır, hareket segmenti Jolt raycast ile taranır.
// Temas halinde material_db üzerinden penetrasyon / sekme / saplanma kararı
// verilir. Sistem deterministiktir (rastgelelik yok) — aynı girdi aynı sonucu
// üretir.
//
// Body -> malzeme eşlemesi: body user data alanındaki MaterialId okunur.

#include "core/JoltAdapter.h"
#include "material_db/MaterialDatabase.h"

#include <Jolt/Jolt.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Body/BodyLockInterface.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/NarrowPhaseQuery.h>
#include <Jolt/Physics/Body/BodyFilter.h>

#include <cmath>
#include <vector>

namespace alazforge {

struct BulletParams {
    float massKg = 0.0095f;          // 9.5 g (7.62x51 tipik)
    float muzzleVelocity = 830.0f;   // m/s
    float caliberMm = 7.62f;
    float dragFactor = 0.00035f;     // 1/m — kuadratik sürükleme katsayısı
    float penetrationRHAMm = 10.0f;  // namlu hızında mm RHA penetrasyon
};

enum class ImpactType : uint8_t {
    Penetration, // malzemeyi delip geçti
    Ricochet,    // yüzeyden sekti
    Embed        // saplandı/durdu
};

struct ImpactEvent {
    ImpactType type;
    Vec3 position;
    Vec3 normal;
    MaterialId material = kInvalidMaterial;
    float impactSpeed = 0.0f;     // m/s, temas anındaki hız
    float remainingSpeed = 0.0f;  // m/s, olay sonrası hız (Embed'de 0)
    float grazingAngleDeg = 0.0f; // yüzeye göre sıyırma açısı
};

struct BulletSimResult {
    std::vector<ImpactEvent> impacts;
    Vec3 finalPosition;
    Vec3 finalVelocity;
    float flightTime = 0.0f;
    bool stopped = false; // saplandı veya hızı tükendi
};

class BallisticsSystem {
public:
    struct Config {
        float gravity = 9.81f;
        float timeStep = 1.0f / 240.0f;   // segment çözünürlüğü
        float minSpeed = 30.0f;           // bu hızın altında mermi etkisiz sayılır
        float maxThicknessProbe = 2.0f;   // m, kalınlık ölçüm sondası menzili
        float ricochetSpeedRetention = 0.55f; // sekmede korunan hız oranı
    };

    BallisticsSystem(JPH::PhysicsSystem& inPhysics, const MaterialDatabase& inMaterials,
                     const Config& inConfig = {})
        : physics(inPhysics), materials(inMaterials), config(inConfig) {}

    // Mermiyi ateşler ve tüm uçuşu simüle eder.
    BulletSimResult Fire(const BulletParams& bullet, const Vec3& origin,
                         const Vec3& direction, float maxFlightTime = 5.0f) {
        BulletSimResult result;

        JPH::Vec3 dir = ToJolt(direction).Normalized();
        JPH::RVec3 pos = ToJoltR(origin);
        JPH::Vec3 vel = dir * bullet.muzzleVelocity;

        float t = 0.0f;
        const float dt = config.timeStep;

        while (t < maxFlightTime) {
            // Yerçekimi düşüşü + kuadratik hava direnci
            vel += JPH::Vec3(0, -config.gravity, 0) * dt;
            const float speed = vel.Length();
            if (speed < config.minSpeed) {
                result.stopped = true;
                break;
            }
            vel -= vel * (bullet.dragFactor * speed * dt);

            // Bu adımın hareket segmentini tara
            const JPH::Vec3 segment = vel * dt;
            JPH::RRayCast ray{pos, segment};
            JPH::RayCastResult hit;
            if (physics.GetNarrowPhaseQuery().CastRay(ray, hit)) {
                const JPH::RVec3 hitPos = ray.GetPointOnRay(hit.mFraction);
                JPH::Vec3 normal;
                MaterialId matId = kInvalidMaterial;
                {
                    JPH::BodyLockRead lock(physics.GetBodyLockInterface(), hit.mBodyID);
                    if (lock.Succeeded()) {
                        const JPH::Body& body = lock.GetBody();
                        normal = body.GetWorldSpaceSurfaceNormal(hit.mSubShapeID2, hitPos);
                        matId = static_cast<MaterialId>(body.GetUserData());
                    }
                }

                if (!ResolveImpact(bullet, hit.mBodyID, hitPos, normal, matId,
                                   pos, vel, result)) {
                    result.stopped = true;
                    break; // saplandı
                }
                t += dt * hit.mFraction;
                continue;
            }

            pos += segment;
            t += dt;
        }

        result.finalPosition = FromJolt(JPH::Vec3(pos));
        result.finalVelocity = FromJolt(vel);
        result.flightTime = t;
        return result;
    }

private:
    JPH::PhysicsSystem& physics;
    const MaterialDatabase& materials;

    Config config;

    // Temas çözümü. Mermi devam ediyorsa true döner (pos/vel güncellenir).
    bool ResolveImpact(const BulletParams& bullet, JPH::BodyID bodyId,
                       const JPH::RVec3& hitPos, const JPH::Vec3& normal,
                       MaterialId matId, JPH::RVec3& pos, JPH::Vec3& vel,
                       BulletSimResult& result) {
        const float speed = vel.Length();
        const JPH::Vec3 dir = vel / speed;

        // Sıyırma açısı: yüzey düzlemi ile mermi yönü arasındaki açı
        const float cosToNormal = std::fabs(dir.Dot(normal));
        const float grazingDeg =
            std::asin(std::min(cosToNormal, 1.0f)) * 57.29578f;

        ImpactEvent ev;
        ev.position = FromJolt(JPH::Vec3(hitPos));
        ev.normal = FromJolt(normal);
        ev.material = matId;
        ev.impactSpeed = speed;
        ev.grazingAngleDeg = grazingDeg;

        const MaterialProperties* mat =
            (matId != kInvalidMaterial && matId < materials.Count())
                ? &materials.Get(matId) : nullptr;

        // ── Sekme kontrolü ──────────────────────────────────────────
        if (mat && mat->ricochetCriticalAngleDeg > 0.0f &&
            grazingDeg < mat->ricochetCriticalAngleDeg) {
            const JPH::Vec3 reflected = dir - 2.0f * dir.Dot(normal) * normal;
            vel = reflected * speed * config.ricochetSpeedRetention;
            pos = hitPos + reflected * 0.001f; // yüzeyden ayır

            ev.type = ImpactType::Ricochet;
            ev.remainingSpeed = vel.Length();
            result.impacts.push_back(ev);
            return true;
        }

        // ── Penetrasyon kontrolü ────────────────────────────────────
        // Kapasite hızın karesiyle ölçeklenir (kinetik enerji ~ v^2)
        const float speedRatio = speed / bullet.muzzleVelocity;
        const float capacityRHAMm = bullet.penetrationRHAMm * speedRatio * speedRatio;

        const float thicknessM = MeasureThickness(bodyId, hitPos, dir);
        const float wallRHAMm =
            mat ? (thicknessM * 100.0f) * mat->rhaEquivalentPerCm : 1e9f;

        if (thicknessM >= 0.0f && capacityRHAMm > wallRHAMm) {
            // Delip geçti: enerji kaybı duvar direnci oranında
            const float exitSpeed = speed * std::sqrt(1.0f - wallRHAMm / capacityRHAMm);
            const JPH::RVec3 exitPoint = hitPos + dir * thicknessM;
            pos = exitPoint + dir * 0.001f;
            vel = dir * exitSpeed;

            ev.type = ImpactType::Penetration;
            ev.remainingSpeed = exitSpeed;
            result.impacts.push_back(ev);
            return exitSpeed >= config.minSpeed;
        }

        // ── Saplandı ────────────────────────────────────────────────
        pos = hitPos;
        vel = JPH::Vec3::sZero();
        ev.type = ImpactType::Embed;
        ev.remainingSpeed = 0.0f;
        result.impacts.push_back(ev);
        return false;
    }

    // Giriş noktasından mermi yönünde gövde kalınlığını ölçer:
    // gövdenin öte tarafından geriye doğru ray atılıp çıkış yüzeyi bulunur.
    // Ölçülemezse (gövde sondadan kalın) -1 döner -> saplanma.
    float MeasureThickness(JPH::BodyID bodyId, const JPH::RVec3& entry,
                           const JPH::Vec3& dir) {
        const float probeLen = config.maxThicknessProbe;
        const JPH::RVec3 probeStart = entry + dir * probeLen;

        JPH::RRayCast backRay{probeStart, -dir * probeLen};
        JPH::RayCastResult backHit;
        // Yalnızca aynı gövdeyi hedefle
        class SameBodyFilter : public JPH::BodyFilter {
        public:
            explicit SameBodyFilter(JPH::BodyID inId) : id(inId) {}
            bool ShouldCollide(const JPH::BodyID& inBodyID) const override {
                return inBodyID == id;
            }
        private:
            JPH::BodyID id;
        } filter(bodyId);

        if (!physics.GetNarrowPhaseQuery().CastRay(
                backRay, backHit, {}, {}, filter))
            return -1.0f;

        // Çıkış noktası: geri ray'in ilk teması
        const float exitDistFromProbe = backHit.mFraction * probeLen;
        const float thickness = probeLen - exitDistFromProbe;
        return thickness;
    }
};

} // namespace alazforge
