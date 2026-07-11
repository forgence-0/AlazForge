#pragma once
// AlazForgeContext — AlazEngine entegrasyonu için TEK giriş noktası (facade).
// Jolt'un yaşam döngüsünü (Factory/RegisterTypes/TempAllocator/
// JobSystemThreadPool/PhysicsSystem::Init) VE dünya başına bir kez kurulması
// gereken tüm "world-scoped" alt sistemleri (rüzgar, sürtünme bölgeleri,
// yüzerlik, LOD, çarpışma-ses olayları, debug çizim; isteğe bağlı olarak
// terrain deform + yıkılabilir yapılar) sarmalar. Amaç: AlazEngine'in bu
// sınıfı bir kez `new` edip her frame `Step()` çağırması yeterli olsun —
// 16 ayrı DLL'i elle birbirine bağlamak zorunda kalmasın.
//
// Bilinçli olarak Context'e DAHİL EDİLMEYENLER (varlık-başına / entity-başına
// olan sistemler, tek bir "dünya" sahibi olamaz): VehicleSystem,
// CharacterController, RopeSystem, RagdollInstance, ClothSystem, AeroSystem,
// TurretMount, weapons.dll'deki tek-atımlık yardımcılar (RecoilImpulse,
// SpreadAccumulator, ExplosionSystem, TrajectoryPredictor). Bunlar oyun
// tarafında varlık başına örneklenir, Physics()'ten aldığı JPH::PhysicsSystem&
// ile kurulur.
//
// AlazEngine'in gerçek ECS'i bu repodan görünmüyor; bu yüzden entity id
// eşlemesi kasıtlı olarak burada tutulmuyor — oyun tarafı BodyID -> entity
// tablosunu kendi sahiplenmeli. AlazEngine'in ECS'i erişilebilir olduğunda bu
// sınıf, doğrudan component senkronizasyonu yapan ince bir köprüye
// dönüştürülebilir; o zamana kadar da tek başına gerçek bir entegrasyon
// noktasıdır (demo/prototip değil) — 25+ testle doğrulanmış alt sistemlerin
// tümünü tek bir Step() arkasında toplar.
//
// JPH::Factory::sInstance process-global bir singleton'dur; bu sınıf
// ref-sayımlı global init/shutdown sahiplenir, böylece birden fazla context
// güvenle (iç içe olmayan sırada) kurulup yıkılabilir.

#include "audio/CollisionEventSystem.h"
#include "ballistics/BallisticsSystem.h"
#include "buoyancy/BuoyancySystem.h"
#include "core/AlazMath.h"
#include "debugdraw/DebugDrawBridge.h"
#include "destructible/DestructibleStructureSystem.h"
#include "lod/LODSystem.h"
#include "material_db/MaterialDatabase.h"
#include "terrain_deform/TerrainDeformSystem.h"
#include "wind/WindSystem.h"
#include "zones/FrictionZoneSystem.h"

#include <context/export.h>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/PhysicsSystem.h>

#include <cstdint>
#include <memory>
#include <vector>

namespace JPH {
class TempAllocatorImpl;
class JobSystemThreadPool;
} // namespace JPH

namespace alazforge {

class BPLayerInterfaceImpl;
class ObjectVsBroadPhaseLayerFilterImpl;
class ObjectLayerPairFilterImpl;

class CONTEXT_API AlazForgeContext {
public:
    struct Config {
        uint32_t maxBodies = 4096;
        uint32_t maxBodyPairs = 4096;
        uint32_t maxContactConstraints = 4096;
        int numThreads = -1; // -1 = hardware_concurrency()-1
        JPH::ObjectLayer movingLayer = 1;
        JPH::ObjectLayer nonMovingLayer = 0;

        WindConfig wind{};
        float collisionEventMinImpactSpeed = 0.5f; // audio olayları icin esik
    };

    // NOT: Config, AlazForgeContext'e iç içe (nested) tanımlı; bu yüzden
    // Config{}'i BURADA varsayılan argüman değeri olarak kullanamayız -- GCC
    // (ve standart), iç içe tipin varsayılan üye ilklendiricilerinin ancak
    // SARAN sınıf tamamlandıktan sonra kullanılabilir olduğunu şart koşuyor
    // (bkz. BallisticsSystem.h'deki aynı not). İki-constructor'a bölüp
    // varsayılanı .cpp'de çözüyoruz.
    AlazForgeContext();
    explicit AlazForgeContext(const Config& inConfig);
    ~AlazForgeContext();

    AlazForgeContext(const AlazForgeContext&) = delete;
    AlazForgeContext& operator=(const AlazForgeContext&) = delete;

    // Fiziği ilerletir VE world-scoped alt sistemleri (rüzgar zamanı,
    // yüzerlik, sürtünme bölgeleri) doğru sırada günceller. lodReferencePoint
    // verilirse (genelde oyuncu/kamera) LODSystem de bu adımda güncellenir;
    // nullptr ise LOD güncellemesi atlanır (çağıran isterse ayrıca
    // LOD().Update(...) çağırabilir).
    void Step(float dt, int collisionSteps = 1, const Vec3* lodReferencePoint = nullptr);

    JPH::PhysicsSystem& Physics();
    BallisticsSystem& Ballistics();
    MaterialDatabase& Materials();

    // ── World-scoped alt sistemler (her zaman mevcut, disk I/O yok) ────
    WindSystem& Wind();
    FrictionZoneSystem& Zones();
    BuoyancySystem& Buoyancy();
    LODSystem& LOD();
    DebugDrawBridge& DebugDraw();

    // Her Step() sonrası (veya ayrı bir noktada) çağrılır: birikmiş
    // çarpışma-ses olaylarını out'a taşır (out önce temizlenir).
    void DrainCollisionEvents(std::vector<CollisionEvent>& out);

    // ── İsteğe bağlı, disk I/O gerektiren alt sistemler ─────────────────
    // Bu ikisi varsayılan olarak KURULMAZ (ChunkStreamSystem, kurulduğu anda
    // savePath dizinini diske yazar — kullanmayan oyunlar için istenmeyen
    // yan etki). İlk çağrıda oluşturulur; sonraki çağrılar cfg'yi yok sayıp
    // mevcut örneği döner (yeniden yapılandırma için önce Reset gerekir).
    TerrainDeformSystem& EnableTerrain(const TerrainDeformSystem::Config& cfg = {});
    DestructibleStructureSystem& EnableDestructible(
        const DestructibleStructureSystem::Config& cfg = {});

    bool HasTerrain() const { return terrain != nullptr; }
    bool HasDestructible() const { return destructible != nullptr; }
    // nullptr döner eğer Enable* hiç çağrılmadıysa -- her frame Enable*
    // çağırıp null kontrolü yapmak yerine bunu kullanmak daha ucuz.
    TerrainDeformSystem* TerrainIfEnabled() { return terrain.get(); }
    DestructibleStructureSystem* DestructibleIfEnabled() { return destructible.get(); }

    struct BodySnapshot {
        uint64_t bodyId;
        Transform transform;
        Vec3 linearVelocity;
        Vec3 angularVelocity;
    };

    // Yalnızca aktif (uykuda olmayan) gövdeler — ECS her frame tüm dünyayı
    // değil, yalnızca hareket edeni senkronize etsin diye.
    size_t SnapshotActiveBodies(std::vector<BodySnapshot>& out) const;

    // ── Dünya durumu kaydet/yükle (oyun save/load sistemi için) ────────
    // Tüm fizik dünyasının (gövde pozisyon/hız/uyku durumları, contact
    // önbelleği) binary snapshot'ı. RestoreWorldState AYNI gövde kümesine
    // sahip bir dünyaya uygulanmalıdır (Jolt'un StateRecorder sözleşmesi:
    // gövdeleri geri EKLEMEZ, mevcut gövdelerin durumunu geri yükler) —
    // yani önce sahneyi kur, sonra state'i yükle. Deterministik yeniden
    // oynatma (replay) için de aynı mekanizma kullanılır.
    size_t SaveWorldState(std::vector<uint8_t>& out) const;
    bool RestoreWorldState(const std::vector<uint8_t>& data);

private:
    Config config;

    std::unique_ptr<JPH::TempAllocatorImpl> tempAllocator;
    std::unique_ptr<JPH::JobSystemThreadPool> jobSystem;
    std::unique_ptr<BPLayerInterfaceImpl> bpLayerInterface;
    std::unique_ptr<ObjectVsBroadPhaseLayerFilterImpl> objectVsBroadPhaseFilter;
    std::unique_ptr<ObjectLayerPairFilterImpl> objectLayerPairFilter;
    std::unique_ptr<JPH::PhysicsSystem> physics;
    std::unique_ptr<MaterialDatabase> materials;
    std::unique_ptr<BallisticsSystem> ballistics;

    std::unique_ptr<WindSystem> wind;
    std::unique_ptr<FrictionZoneSystem> zones;
    std::unique_ptr<BuoyancySystem> buoyancy;
    std::unique_ptr<LODSystem> lod;
    std::unique_ptr<CollisionEventSystem> collisionEvents;
    std::unique_ptr<DebugDrawBridge> debugDraw;

    std::unique_ptr<TerrainDeformSystem> terrain;
    std::unique_ptr<DestructibleStructureSystem> destructible;
};

} // namespace alazforge
