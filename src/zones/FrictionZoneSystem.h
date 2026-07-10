#pragma once
// Sürtünme bölgeleri: buz, çamur, yağ gibi bölgesel zemin koşulları.
// Eksene hizalı kutu bölgeler; izlenen bir gövde bölgeye girince sürtünme/
// sekme değerleri bölgeninkiyle değiştirilir, çıkınca orijinali geri yüklenir.
//
// Araç entegrasyonu: Jolt'un tekerlek modeli, lastik sürtünmesini temas
// edilen ZEMİN gövdesinin friction'ıyla çarpar (VehicleConstraint'in
// varsayılan CombineFriction'ı) — zemin gövdesini TrackBody ile izleyip
// bölgeden geçirmek yerine, zemini parça parça gövdeler olarak kurup her
// parçaya MaterialDatabase::ApplyToBody("ice") uygulamak da aynı sonucu
// verir. QueryFrictionAt ise oyun tarafının özel CombineFriction callback'i
// yazmasına imkân tanır (tek parça dev zeminde bölgesel buz için).

#include "core/AlazMath.h"

#include <zones/export.h>

#include <Jolt/Jolt.h>

#include <Jolt/Physics/PhysicsSystem.h>

#include <vector>

namespace alazforge {

struct FrictionZone {
    Vec3 boundsMin{0, 0, 0};
    Vec3 boundsMax{0, 0, 0};
    float friction = 0.05f;    // bölge içindeki sürtünme (varsayılan: buz)
    float restitution = -1.0f; // <0 = gövdenin kendi değeri korunur
};

class ZONES_API FrictionZoneSystem {
public:
    // Bölge ekler; kararlı index döner
    size_t AddZone(const FrictionZone& zone);
    void RemoveZone(size_t index);

    // Gövdeyi izlemeye al: o anki friction/restitution "orijinal" olarak
    // saklanır, bölge dışına çıkınca geri yüklenir.
    void TrackBody(JPH::PhysicsSystem& physics, JPH::BodyID body);
    void UntrackBody(JPH::PhysicsSystem& physics, JPH::BodyID body);

    // Her frame: izlenen gövdelerin bölge içi/dışı durumunu uygular
    void Update(JPH::PhysicsSystem& physics);

    // Noktadaki etkin sürtünme (bölge yoksa defaultFriction döner) —
    // özel araç CombineFriction callback'leri için.
    float QueryFrictionAt(const Vec3& worldPos, float defaultFriction) const;

private:
    struct StoredZone {
        FrictionZone zone;
        bool active = false;
    };
    struct TrackedBody {
        JPH::BodyID body;
        float originalFriction = 0.5f;
        float originalRestitution = 0.0f;
        bool inZone = false;
    };

    const FrictionZone* FindZoneContaining(const Vec3& worldPos) const;

    std::vector<StoredZone> zones;
    std::vector<TrackedBody> tracked;
};

} // namespace alazforge
