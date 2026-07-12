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

#include <physics/export.h>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/PhysicsSystem.h>

#include <cstdint>
#include <vector>

namespace alazforge {

struct BulletParams {
    float massKg = 0.0095f;        // 9.5 g (7.62x51 tipik)
    float muzzleVelocity = 830.0f; // m/s
    float caliberMm = 7.62f;
    float dragFactor = 0.00035f;    // 1/m — kuadratik sürükleme katsayısı
    float penetrationRHAMm = 10.0f; // namlu hızında mm RHA penetrasyon
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
    // Vurulan gövdenin id'si — DestructibleStructureSystem::FindPieceByBodyID
    // gibi tüketicilerin vuruşu bir parçaya/nesneye bağlayabilmesi için.
    JPH::BodyID hitBody;
};

struct BulletSimResult {
    std::vector<ImpactEvent> impacts;
    Vec3 finalPosition;
    Vec3 finalVelocity;
    float flightTime = 0.0f;
    bool stopped = false; // saplandı veya hızı tükendi
};

class PHYSICS_API BallisticsSystem {
public:
    struct Config {
        float gravity = 9.81f;
        float timeStep = 1.0f / 240.0f;       // segment çözünürlüğü
        float minSpeed = 30.0f;               // bu hızın altında mermi etkisiz sayılır
        float maxThicknessProbe = 2.0f;       // m, kalınlık ölçüm sondası menzili
        float ricochetSpeedRetention = 0.55f; // sekmede korunan hız oranı
    };

    // NOT: Config, BallisticsSystem'e iç içe (nested) tanımlı; bu yüzden
    // Config{}'i BURADA (sınıf gövdesi içinde) varsayılan argüman değeri
    // olarak kullanamayız -- GCC (ve standart), iç içe tipin varsayılan üye
    // ilklendiricilerinin ancak SARAN sınıf tamamlandıktan sonra kullanılabilir
    // olduğunu şart koşuyor (döngüsel bağımlılık: BallisticsSystem, kendi
    // içindeki bir varsayılan argüman için henüz tamamlanmamış Config'e
    // ihtiyaç duyar). İki-constructor'a bölüp varsayılanı .cpp'de (çeviri
    // birimi kapsamında, her iki tip de tam tanımlıyken) çözüyoruz.
    BallisticsSystem(JPH::PhysicsSystem& inPhysics, const MaterialDatabase& inMaterials);
    BallisticsSystem(JPH::PhysicsSystem& inPhysics, const MaterialDatabase& inMaterials,
                     const Config& inConfig);

    // Mermiyi ateşler ve tüm uçuşu simüle eder.
    BulletSimResult Fire(const BulletParams& bullet, const Vec3& origin, const Vec3& direction,
                         float maxFlightTime = 5.0f);

private:
    JPH::PhysicsSystem& physics;
    const MaterialDatabase& materials;

    Config config;

    // Temas çözümü. Mermi devam ediyorsa true döner (pos/vel güncellenir).
    bool ResolveImpact(const BulletParams& bullet, JPH::BodyID bodyId, const JPH::RVec3& hitPos,
                       const JPH::Vec3& normal, MaterialId matId, JPH::RVec3& pos, JPH::Vec3& vel,
                       BulletSimResult& result);

    // Giriş noktasından mermi yönünde gövde kalınlığını ölçer:
    // gövdenin öte tarafından geriye doğru ray atılıp çıkış yüzeyi bulunur.
    // Ölçülemezse (gövde sondadan kalın) -1 döner -> saplanma.
    float MeasureThickness(JPH::BodyID bodyId, const JPH::RVec3& entry, const JPH::Vec3& dir);
};

} // namespace alazforge
