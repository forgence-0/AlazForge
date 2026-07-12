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

    // Segmentler arası top eklemin kopma eşiği (Newton). Gerginlik,
    // JPH::PointConstraint'in son adımda çözdüğü impuls büyüklüğünden
    // (lambda/dt ~ kuvvet) okunur — Jolt'un kendi breakable-constraint
    // örneklerinde kullanılan yöntem. 0 = kopmaz (varsayılan, geriye
    // uyumlu). Uç bağlantıları (AttachStartTo/AttachEndTo) bu kontrole
    // dahil değil, yalnızca segmentler arası eklemler kopabilir.
    float maxTensionN = 0.0f;
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

    // Belirli bir segmentin gövde id'si -- örn. dışarıdan impulse/kuvvet
    // uygulamak (bir cisim halatı yakalayıp çekmek) veya BodyInterface
    // sorguları için. Aralık dışında geçersiz JPH::BodyID döner.
    JPH::BodyID GetSegmentBodyID(int index) const;

    // Her fizik adımından SONRA çağrılır: config.maxTensionN'i aşan
    // segment eklemlerini fizikten kaldırır (halat gerçekten ikiye
    // ayrılır — parçalar artık serbestçe düşer). outBrokenJoints
    // verilirse kırılan eklem indeksleri (0..SegmentCount()-2) eklenir.
    // maxTensionN <= 0 ise no-op (kopmaz halat, varsayılan).
    void Update(float dt, std::vector<int>* outBrokenJoints = nullptr);

    // jointIndex'teki segmentler-arası eklem koptu mu (veya aralık dışı).
    bool IsJointBroken(int jointIndex) const;

    void Destroy();

private:
    void AttachTo(JPH::BodyID segment, JPH::BodyID target, const Vec3& worldAnchor);

    RopeConfig config;
    JPH::PhysicsSystem* physics = nullptr;
    std::vector<JPH::BodyID> segments;
    std::vector<JPH::Ref<JPH::Constraint>> constraints;
    // constraints[0..segmentJointCount) = segmentler arası eklemler (kopabilir);
    // bunun ötesi AttachStartTo/AttachEndTo'dan gelen uç bağlantıları.
    size_t segmentJointCount = 0;
};

} // namespace alazforge
