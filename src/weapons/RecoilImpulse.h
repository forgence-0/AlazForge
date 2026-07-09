#pragma once
// Geri tepme (recoil) hesaplama (Faz B.1).
//
// Ateş anında merminin kazandığı momentumun tersi, taşıyıcı gövdeye (araç
// gövdesi veya türet mount'un ara gövdesi) impulse olarak uygulanır — saf
// momentum korunumu, ekstra durum/yaşam döngüsü gerektirmez. BallisticsSystem
// ::Fire'a dokunulmaz; çağıran taraf mermiyi ateşlerken bu fonksiyonu ayrıca
// çağırır.

#include "core/JoltAdapter.h"

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/PhysicsSystem.h>

namespace alazforge {

struct RecoilParams {
    float bulletMassKg = 0.0f;
    float muzzleVelocity = 0.0f;
    float recoilMultiplier = 1.0f;    // gaz sistemi/geri tepme mekanizması farkı
    float muzzleClimbTorqueNm = 0.0f; // 0 = namlu kalkışı yok (açısal impulse, N*m*s)
};

class RecoilImpulse {
public:
    // Momentum korunumu: impulse, mermi yönünün tersine, mermi momentumu
    // (kütle * namlu hızı) kadar — recoilMultiplier ile ölçeklenir.
    static Vec3 ComputeImpulse(const RecoilParams& params, const Vec3& fireDirection) {
        const JPH::Vec3 dir = ToJolt(fireDirection).Normalized();
        const JPH::Vec3 impulse =
            -dir * (params.bulletMassKg * params.muzzleVelocity * params.recoilMultiplier);
        return FromJolt(impulse);
    }

    // Taşıyıcı gövdeye, ateşleme noktasında lineer impulse + (varsa) namlu
    // kalkışı için gövdenin yerel sağ ekseni etrafında açısal impulse uygular.
    static void ApplyToBody(JPH::PhysicsSystem& physics, JPH::BodyID carrierBody,
                            const RecoilParams& params, const Vec3& fireDirection,
                            const Vec3& applicationPointWorld) {
        JPH::BodyInterface& bi = physics.GetBodyInterface();
        const JPH::Vec3 impulse = ToJolt(ComputeImpulse(params, fireDirection));
        bi.AddImpulse(carrierBody, impulse, ToJoltR(applicationPointWorld));

        if (params.muzzleClimbTorqueNm != 0.0f) {
            const JPH::Quat rot = bi.GetRotation(carrierBody);
            const JPH::Vec3 rightAxis = rot * JPH::Vec3(1, 0, 0);
            bi.AddAngularImpulse(carrierBody, rightAxis * params.muzzleClimbTorqueNm);
        }
    }
};

} // namespace alazforge
