#pragma once
// Türet/silah mount fiziği (Faz B.3).
//
// Türet, ana gövdeye (araç şasisi) motorlu iki JPH::HingeConstraint ile
// bağlanan iki küçük ara gövde üzerinden simüle edilir: yaw platformu
// (dikey eksen etrafında) ve onun üstünde oturan pitch/namlu gövdesi (yatay
// eksen etrafında). Çarpışmayla fiziksel olarak sıkışabilir/zorlanabilir —
// istenen davranış bu (kinematik açısal entegratör değil, gerçek constraint).
// Motor hız modunda çalışır: SetTargetAngles her çağrıldığında açı farkına
// göre orantılı bir hedef açısal hız hesaplanır, yapılandırılan
// yawRateDegPerSec/pitchRateDegPerSec'e kenetlenir.
//
// Yaw/pitch ara gövdeleri ile ana gövde (parentBody) arasında, ve yaw ile
// pitch arasında JPH::GroupFilterTable ile çarpışma devre dışı bırakılır --
// aksi halde (gerçek CI'da yakalandı) türet pivotu ana gövdenin shape'ine
// dokunuyor/gömülüyorsa, motorun çalıştığı eksene bağlı olarak sürekli bir
// penetrasyon-düzeltme torku motor komutunu tamamen bastırabiliyordu.

#include "core/JoltAdapter.h"

#include <weapons/export.h>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Constraints/HingeConstraint.h>
#include <Jolt/Physics/PhysicsSystem.h>

namespace alazforge {

struct TurretMountConfig {
    Vec3 localOffset;       // gövdeye göre kule pivotu (gövde yerel uzayı)
    Vec3 localMuzzleOffset; // pivottan namlu ucuna (namlu gövdesi yerel uzayında, +z ileri)
    float yawMotorMaxTorqueNm = 5000.0f;
    float pitchMotorMaxTorqueNm = 3000.0f;
    float yawRateDegPerSec = 60.0f;
    float pitchRateDegPerSec = 30.0f;
    bool yawLimited = false;
    float minYawDeg = -180.0f, maxYawDeg = 180.0f;
    float minPitchDeg = -10.0f, maxPitchDeg = 20.0f;
};

class WEAPONS_API TurretMount {
public:
    explicit TurretMount(const TurretMountConfig& inConfig);
    ~TurretMount();

    TurretMount(const TurretMount&) = delete;
    TurretMount& operator=(const TurretMount&) = delete;

    // Ana gövdeye (örn. araç şasisi) yaw+pitch constraint zincirini kurar.
    void AttachToBody(JPH::PhysicsSystem& inPhysics, JPH::BodyID parentBody,
                      JPH::ObjectLayer layer);

    // Hedef yaw/pitch açısına (radyan, dünya değil parent'a göre) motor
    // kontrolüyle yönlendirir. Her frame çağrılması beklenir (VehicleSystem
    // ::SetDriverInput ile aynı kullanım deseni).
    void SetTargetAngles(float targetYawRad, float targetPitchRad);

    float CurrentYaw() const;
    float CurrentPitch() const;

    // Namlu ucunun dünya uzayındaki pozisyon+rotasyonu — BallisticsSystem::
    // Fire'a doğrudan geçirilebilir (Forward(transform.rotation) ateş yönü).
    Transform GetMuzzleWorldTransform() const;

    void Destroy();

private:
    TurretMountConfig config;
    JPH::PhysicsSystem* physics = nullptr;
    JPH::BodyID yawBody;
    JPH::BodyID pitchBody;
    JPH::Ref<JPH::HingeConstraint> yawConstraint;
    JPH::Ref<JPH::HingeConstraint> pitchConstraint;
};

} // namespace alazforge
