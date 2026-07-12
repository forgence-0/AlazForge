// Örnek entegrasyon uygulaması: AlazForgeContext'i AlazEngine'in yapacağı
// gibi kullanan minimal, GERÇEKTEN ÇALIŞAN bir konsol demosu. Amaç bir
// "merhaba dünya" degil -- entegrasyonun ihtiyaç duyacağı gerçek akışı
// uçtan uca göstermek: sahne kurulumu + malzeme ataması, balistik ateşleme,
// çarpışma-ses olaylarını okuma, sorgu API'si (raycast), her frame aktif
// gövde snapshot'ı alma, ve dünya durumu kaydet/geri yükle.
//
// Çalıştırma: build/examples/alazforge_sample_integration

#include "context/AlazForgeContext.h"
#include "core/JoltAdapter.h"
#include "scene/QueryHelpers.h"

#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>

#include <cstdio>

using namespace alazforge;

namespace {

void PrintTransform(const char* label, const Transform& t) {
    printf("    %-10s pos=(%.2f, %.2f, %.2f)\n", label, t.position.x, t.position.y, t.position.z);
}

} // namespace

int main() {
    printf("=== AlazForge Ornek Entegrasyon Uygulamasi ===\n\n");

    // 1) Context: Jolt yasam dongusu + world-scoped alt sistemler tek
    //    Step() arkasinda. AlazEngine gercekte bunu bir kez olusturup
    //    sahiplenir.
    AlazForgeContext context;
    JPH::PhysicsSystem& physics = context.Physics();
    JPH::BodyInterface& bodyInterface = physics.GetBodyInterface();

    // 2) Sahne: zemin (beton, statik) + ahsap sandik (dinamik) + celik kure
    const MaterialId concrete = context.Materials().FindByName("concrete");
    const MaterialId wood = context.Materials().FindByName("wood");
    const MaterialId steel = context.Materials().FindByName("steel");

    JPH::BodyCreationSettings groundSettings(new JPH::BoxShape(JPH::Vec3(20, 0.5f, 20)),
                                             JPH::RVec3(0, -0.5, 0), JPH::Quat::sIdentity(),
                                             JPH::EMotionType::Static, /*layer=*/0);
    const JPH::BodyID ground =
        bodyInterface.CreateAndAddBody(groundSettings, JPH::EActivation::DontActivate);
    context.Materials().ApplyToBody(bodyInterface, ground, concrete);

    JPH::BodyCreationSettings crateSettings(new JPH::BoxShape(JPH::Vec3(0.5f, 0.5f, 0.5f)),
                                            JPH::RVec3(0, 5, 0), JPH::Quat::sIdentity(),
                                            JPH::EMotionType::Dynamic, 1);
    const JPH::BodyID crate =
        bodyInterface.CreateAndAddBody(crateSettings, JPH::EActivation::Activate);
    context.Materials().ApplyToBody(bodyInterface, crate, wood);

    JPH::BodyCreationSettings ballSettings(new JPH::SphereShape(0.3f), JPH::RVec3(3, 5, 0),
                                           JPH::Quat::sIdentity(), JPH::EMotionType::Dynamic, 1);
    const JPH::BodyID ball =
        bodyInterface.CreateAndAddBody(ballSettings, JPH::EActivation::Activate);
    context.Materials().ApplyToBody(bodyInterface, ball, steel);

    physics.OptimizeBroadPhase();
    printf("Sahne kuruldu: 1 zemin (beton) + 1 sandik (ahsap) + 1 kure (celik)\n\n");

    // 3) Balistik: sandigi yukaridan vur, sonucu yazdir.
    printf("Balistik ates: sandiga yukaridan atis...\n");
    const BulletSimResult shot =
        context.Ballistics().Fire(BulletParams{}, Vec3{0, 10, 0}, Vec3{0, -1, 0});
    for (const ImpactEvent& impact : shot.impacts) {
        const char* typeName = impact.type == ImpactType::Penetration ? "delip-gecti"
                               : impact.type == ImpactType::Ricochet  ? "sekti"
                                                                      : "saplandi";
        printf("  carpisma: %s, hiz=%.1f m/s, konum=(%.2f, %.2f, %.2f)\n", typeName,
               impact.impactSpeed, impact.position.x, impact.position.y, impact.position.z);
    }
    if (shot.impacts.empty()) printf("  (carpisma yok)\n");
    printf("\n");

    // 4) Sorgu API'si: sandigin hemen altinda ne var?
    const RaycastHit queryHit = RaycastClosest(physics, Vec3{0, 3, 0}, Vec3{0, -1, 0}, 10.0f);
    if (queryHit.hit) {
        printf("Sorgu: sandigin altindaki en yakin govde y=%.2f konumunda bulundu\n\n",
               queryHit.point.y);
    }

    // 5) Simulasyonu ilerlet: her frame carpisma-ses olaylarini oku.
    printf("Simulasyon calisiyor (120 frame, ~2 saniye)...\n");
    std::vector<CollisionEvent> events;
    for (int frame = 0; frame < 120; ++frame) {
        context.Step(1.0f / 60.0f);
        context.DrainCollisionEvents(events);
        for (const CollisionEvent& e : events) {
            printf("  [frame %3d] carpisma-ses olayi: hiz=%.1f m/s\n", frame, e.impactSpeed);
        }
    }
    printf("\n");

    // 6) Aktif govde snapshot'i (ECS senkronizasyonunun yaptigi is).
    std::vector<AlazForgeContext::BodySnapshot> snapshot;
    context.SnapshotActiveBodies(snapshot);
    printf("120 frame sonra aktif (uyumayan) govde sayisi: %zu\n", snapshot.size());

    // 7) Dunya durumu kaydet/geri yukle (oyun save/load akisi).
    std::vector<uint8_t> savedState;
    const size_t bytesWritten = context.SaveWorldState(savedState);
    printf("Dunya durumu kaydedildi: %zu bayt\n", bytesWritten);

    Transform crateBeforeMoreSteps;
    {
        JPH::BodyLockRead lock(physics.GetBodyLockInterface(), crate);
        crateBeforeMoreSteps.position = FromJolt(JPH::Vec3(lock.GetBody().GetPosition()));
    }

    // Birkac adim daha ilerlet, sonra kaydedilen duruma geri don.
    for (int i = 0; i < 30; ++i)
        context.Step(1.0f / 60.0f);
    const bool restored = context.RestoreWorldState(savedState);
    printf("Dunya durumu geri yuklendi: %s\n", restored ? "basarili" : "BASARISIZ");

    Transform crateAfterRestore;
    {
        JPH::BodyLockRead lock(physics.GetBodyLockInterface(), crate);
        crateAfterRestore.position = FromJolt(JPH::Vec3(lock.GetBody().GetPosition()));
    }
    printf("  sandik konumu (kayittan hemen sonra): ");
    PrintTransform("crate", crateBeforeMoreSteps);
    printf("  sandik konumu (geri yuklemeden sonra): ");
    PrintTransform("crate", crateAfterRestore);

    printf("\n=== Demo tamamlandi ===\n");
    return 0;
}
