#pragma once
// LOD (detay seviyesi) sistemi: büyük açık dünya sahnelerinde referans
// noktasından (genelde oyuncu) uzak dinamik gövdeleri uyutarak CPU
// tasarrufu sağlayan genel amaçlı yardımcı. Belirli bir sisteme
// (araç/karakter/enkaz) özel değildir — herhangi bir BodyID izlenebilir.
//
// Yaklaşım: Jolt zaten hareketsiz gövdeleri otomatik uyutur (sleep), ama
// bu yalnızca gövde fiilen durunca olur — uzaktaki bir araç motoru
// çalışırken sürekli aktif kalabilir. LODSystem, mesafeye göre gövdeleri
// ZORLA uyutur/uyandırır: yakın gövdeler her zaman tam simüle edilir,
// uzaktakiler donar (fizik güncellenmez, son bilinen durumda kalır).
//
// Histerezis (sleepDistance > wakeDistance değil, tam tersi: uyanma
// mesafesi uyuma mesafesinden KÜÇÜK) sınır çizgisinde titremeyi önler.

#include "core/AlazMath.h"

#include <lod/export.h>

#include <Jolt/Jolt.h>

#include <Jolt/Physics/PhysicsSystem.h>

#include <vector>

namespace alazforge {

struct LODConfig {
    float sleepDistance = 150.0f; // bu mesafenin ötesinde gövde uyutulur
    float wakeDistance = 120.0f;  // bu mesafenin içine girince uyandırılır
};

class LOD_API LODSystem {
public:
    explicit LODSystem(const LODConfig& inConfig);

    void TrackBody(JPH::BodyID body);
    void UntrackBody(JPH::PhysicsSystem& physics, JPH::BodyID body);

    // referencePoint: genelde oyuncu/kamera pozisyonu. Her frame (veya
    // birkaç frame'de bir — sık çağrılması şart değil) çağrılır.
    void Update(JPH::PhysicsSystem& physics, const Vec3& referencePoint);

    size_t TrackedCount() const { return tracked.size(); }
    size_t SleepingCount() const;

private:
    struct TrackedBody {
        JPH::BodyID body;
        bool forcedSleep = false;
    };

    LODConfig config;
    std::vector<TrackedBody> tracked;
};

} // namespace alazforge
