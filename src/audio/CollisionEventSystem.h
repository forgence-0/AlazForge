#pragma once
// Ses tetikleyicileri: her fizik çarpışması, oyun tarafının hangi sesi ne
// şiddette çalacağına karar verebilmesi için bir olay listesine yazılır.
// AlazForge kendi ses çalma sistemini içermez — yalnızca "ne oldu, ne
// kadar şiddetli, hangi malzemeler" bilgisini üretir.
//
// Kullanım: bir kez Attach(physics) çağır (Jolt'un tek bir global
// ContactListener'ı olur — SetContactListener'ı ELE GEÇİRİR, başka bir
// sistem zaten kendi listener'ını kurmuşsa çakışır), her frame sonunda
// DrainEvents ile birikmiş olayları al ve temizle.
//
// Şiddet ölçütü: çarpışma normali doğrultusundaki bağıl hız (m/s) —
// yüzeye paralel sürtünme temaslarını (araç lastiği yolda) filtrelemek
// için düşük bir eşik (minImpactSpeed) altındaki temaslar yok sayılır.

#include "core/JoltAdapter.h"
#include "material_db/MaterialDatabase.h"

#include <audio/export.h>

#include <Jolt/Jolt.h>

#include <Jolt/Physics/Collision/ContactListener.h>
#include <Jolt/Physics/PhysicsSystem.h>

#include <mutex>
#include <vector>

namespace alazforge {

struct CollisionEvent {
    JPH::BodyID bodyA;
    JPH::BodyID bodyB;
    Vec3 worldPosition;
    Vec3 worldNormal;
    float impactSpeed = 0.0f; // normal doğrultusundaki bağıl hız (m/s)
    MaterialId materialA = kInvalidMaterial;
    MaterialId materialB = kInvalidMaterial;
};

class AUDIO_API CollisionEventSystem final : public JPH::ContactListener {
public:
    explicit CollisionEventSystem(float minImpactSpeed = 0.5f);

    // physics->SetContactListener(this) çağırır — süreç boyunca yalnızca
    // bir sistem bunu yapabilir (Jolt'un tek global listener kısıtı).
    void Attach(JPH::PhysicsSystem& physics);

    // Birikmiş olayları out'a taşır (out önce temizlenir) ve dahili
    // kuyruğu boşaltır. Her frame sonunda bir kez çağrılması beklenir.
    void DrainEvents(std::vector<CollisionEvent>& out);

    void OnContactAdded(const JPH::Body& body1, const JPH::Body& body2,
                        const JPH::ContactManifold& manifold,
                        JPH::ContactSettings& settings) override;

private:
    float minImpactSpeed;
    std::mutex mutex; // OnContactAdded fizik iş parçacıklarından çağrılabilir
    std::vector<CollisionEvent> pending;
};

} // namespace alazforge
