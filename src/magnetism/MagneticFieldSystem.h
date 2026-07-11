#pragma once
// Manyetik/çekim alanları: bir noktadan yarıçap içindeki İZLENEN gövdelere
// merkeze doğru (çekme) veya merkezden dışarı (itme) kuvvet uygular.
// BuoyancySystem/FrictionZoneSystem ile aynı desen: yalnızca TrackBody ile
// açıkça işaretlenen gövdeler etkilenir (her metal kutuyu otomatik
// etkilemeye çalışmak yerine, oyun tarafı hangi gövdelerin "manyetik
// duyarlı" olduğuna karar verir — örn. yalnızca demir/çelik malzemeli
// gövdeler TrackBody ile eklenebilir).
//
// Kuvvet modeli: F = strength / max(mesafe^2, minDistSq) — ters kare
// (gerçek manyetik/kütle çekimi yaklaşımı), merkeze/merkezden radyal.
// Deterministik (RNG yok).

#include "core/AlazMath.h"

#include <magnetism/export.h>

#include <Jolt/Jolt.h>

#include <Jolt/Physics/PhysicsSystem.h>

#include <vector>

namespace alazforge {

struct MagneticFieldSource {
    Vec3 position{0, 0, 0};
    float radius = 10.0f;     // bu mesafenin ötesinde etkisiz
    float strength = 500.0f;  // N·m^2 cinsinden (F = strength / mesafe^2)
    float minDistance = 0.5f; // bu mesafenin altında kuvvet doygunlaşır (sonsuza gitmez)
    bool repel = false;       // true = itme, false = çekme (varsayılan)
};

class MAGNETISM_API MagneticFieldSystem {
public:
    // Kaynak ekler; kararlı bir index döner (RemoveSource sonrası diğer
    // index'ler geçerliliğini korur).
    size_t AddSource(const MagneticFieldSource& source);
    void RemoveSource(size_t index);

    void TrackBody(JPH::BodyID body);
    void UntrackBody(JPH::BodyID body);

    // İzlenen her gövdeye, etki alanındaki TÜM kaynakların kuvvetini
    // toplayıp uygular (birden fazla kaynak aynı gövdeyi etkileyebilir).
    // JPH::BodyInterface::AddForce kullanır -- dt gerekmez, Jolt bir
    // sonraki physics.Update() adımında entegre eder.
    void Update(JPH::PhysicsSystem& physics);

private:
    struct StoredSource {
        MagneticFieldSource source;
        bool active = false;
    };

    std::vector<StoredSource> sources;
    std::vector<JPH::BodyID> trackedBodies;
};

} // namespace alazforge
