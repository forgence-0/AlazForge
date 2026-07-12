#pragma once
// AlazForgeContext (Faz B.7) — AlazEngine için gelecekteki ECS bridge
// placeholder'ı. Jolt'un yaşam döngüsünü (Factory/RegisterTypes/
// TempAllocator/JobSystemThreadPool/PhysicsSystem::Init — bugüne kadar her
// testte tests/JoltTestWorld.h içinde tekrar eden kurulum) ve tüm alt
// sistemleri (ballistics, material db) sarmalayan facade.
//
// AlazEngine'in gerçek ECS'i bu repodan görünmüyor; bu yüzden entity id
// eşlemesi kasıtlı olarak burada tutulmuyor — oyun tarafı BodyID -> entity
// tablosunu kendi sahiplenmeli. AlazEngine'in ECS'i erişilebilir olduğunda bu
// sınıf, doğrudan component senkronizasyonu yapan ince bir köprüye
// dönüştürülebilir.
//
// JPH::Factory::sInstance process-global bir singleton'dur; bu sınıf
// ref-sayımlı global init/shutdown sahiplenir, böylece birden fazla context
// güvenle (iç içe olmayan sırada) kurulup yıkılabilir.

#include "ballistics/BallisticsSystem.h"
#include "core/AlazMath.h"
#include "material_db/MaterialDatabase.h"

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

    void Step(float dt, int collisionSteps = 1);

    JPH::PhysicsSystem& Physics();
    BallisticsSystem& Ballistics();
    MaterialDatabase& Materials();

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
};

} // namespace alazforge
