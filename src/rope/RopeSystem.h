#pragma once
// Halat/kablo fiziği: kapsül segment zinciri, segmentler arası
// JPH::PointConstraint (top eklem) ile bağlanır. Vinç halatı, çekme
// halatı, asma köprü gibi kullanımlar için — uçlar isteğe bağlı olarak
// dünyadaki gerçek gövdelere (araç şasisi vb.) sabitlenebilir.
//
// Segment tabanlı model (soft body yerine) seçildi çünkü: segmentler
// gerçek rigid body'dir — araç sistemine takılıp ÇEKEBİLİR, üzerine ağırlık
// binebilir, destructible parçalarına çarpabilir. Kod tabanının geri
// kalanıyla (VehicleSystem/TurretMount/Ragdoll) aynı doğrulanmış
// CreateAndAddBody + constraint deseni kullanılır.

#include "core/JoltAdapter.h"

#include <rope/export.h>

#include <Jolt/Jolt.h>

#include <Jolt/Physics/Constraints/Constraint.h>
#include <Jolt/Physics/PhysicsSystem.h>

#include <vector>

namespace alazforge {

struct RopeConfig {
    int segmentCount = 16;     // segment sayısı (fazlası daha yumuşak bükülür)
    float totalLength = 8.0f;  // toplam halat uzunluğu (m)
    float radius = 0.03f;      // halat yarıçapı (m)
    float massPerMeter = 0.5f; // kg/m (çelik halat ~0.4-4 arası)
};

class ROPE_API RopeSystem {
public:
    explicit RopeSystem(const RopeConfig& inConfig);
    ~RopeSystem();

    RopeSystem(const RopeSystem&) = delete;
    RopeSystem& operator=(const RopeSystem&) = delete;

    // Halatı startPos'tan endPos yönüne doğru düz bir zincir olarak kurar.
    // Kendi haline bırakılırsa yerçekimiyle sarkar/düşer.
    void Create(JPH::PhysicsSystem& inPhysics, const Vec3& startPos, const Vec3& endPos,
                JPH::ObjectLayer layer);

    // Halatın ilk/son segmentini dünyadaki bir gövdeye sabitler (top eklem,
    // worldAnchor: eklem noktası dünya uzayında). Create'ten sonra çağrılır.
    void AttachStartTo(JPH::BodyID body, const Vec3& worldAnchor);
    void AttachEndTo(JPH::BodyID body, const Vec3& worldAnchor);

    int SegmentCount() const { return static_cast<int>(segments.size()); }
    Transform GetSegmentTransform(int index) const; // render için

    void Destroy();

private:
    void AttachTo(JPH::BodyID segment, JPH::BodyID target, const Vec3& worldAnchor);

    RopeConfig config;
    JPH::PhysicsSystem* physics = nullptr;
    std::vector<JPH::BodyID> segments;
    std::vector<JPH::Ref<JPH::Constraint>> constraints;
};

} // namespace alazforge
