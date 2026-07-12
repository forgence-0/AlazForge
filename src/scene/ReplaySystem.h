#pragma once
// Deterministik girdi kaydı/tekrar oynatma. AlazForge'un fizik simülasyonu
// zaten deterministiktir (bkz. `world_state` testi — aynı başlangıç
// durumundan aynı dt dizisiyle adımlamak bit-birebir aynı sonucu verir).
// Bu yüzden HER FRAME'İN TAM DURUMUNU kaydetmek yerine yalnızca DIŞARIDAN
// uygulanan girdiler (impulse'lar) + kullanılan dt kaydedilir -- çok daha
// küçük bir kayıt, çünkü fizik motorunun kendisi tekrar oynatmada aynı
// sonucu deterministik olarak yeniden üretir. Periyodik tam-durum kontrol
// noktaları (checkpoint), kayıttan ortadan başlayarak (sıfırdan yeniden
// simüle etmeden) hızlı ileri sarmayı/geri sarmayı mümkün kılar.
//
// Kullanım alanları: hata ayıklama (bir çöküşü/garip davranışı tekrar
// oynatmak), gelecekteki network rollback (sunucudan düzeltme geldiğinde
// yerel simülasyonu bir önceki kontrol noktasına sarıp girdileri yeniden
// uygulamak).
//
// Not: Checkpoint'ler JPH::BodyID'yi doğrudan saklar -- bu, AYNI çalışan
// PhysicsSystem içinde geri sarma (canlı rollback) VE aynı gövde oluşturma
// sırasıyla YENİDEN kurulmuş taze bir PhysicsSystem'de tekrar oynatma
// (Jolt'un gövde indeks tahsisi deterministiktir, `world_state` testi
// tarafından zaten kanıtlanmıştır) için geçerlidir.

#include "core/AlazMath.h"

#include <physics/export.h>

#include <Jolt/Jolt.h>

#include <Jolt/Physics/Body/BodyID.h>
#include <Jolt/Physics/PhysicsSystem.h>

#include <cstdint>
#include <string>
#include <vector>

namespace alazforge {

struct RecordedImpulse {
    uint32_t frameIndex = 0;
    JPH::BodyID body;
    Vec3 impulse{0, 0, 0};
};

struct RecordedBodyState {
    JPH::BodyID body;
    Vec3 position{0, 0, 0};
    Quat rotation{0, 0, 0, 1};
    Vec3 linearVelocity{0, 0, 0};
    Vec3 angularVelocity{0, 0, 0};
};

struct ReplayCheckpoint {
    uint32_t frameIndex = 0;
    std::vector<RecordedBodyState> bodies;
};

class PHYSICS_API ReplayRecorder {
public:
    // Şu anki frame'de uygulanan bir dış impulse'ı kaydeder (fiziğe
    // uygulandığı yerde çağrılır -- Step()'ten önce ya da sonra olabilir,
    // önemli olan EndFrame ile aynı frame numarasında olması).
    void RecordImpulse(JPH::BodyID body, const Vec3& impulse);

    // Bir frame'in tamamlandığını işaretler (kullanılan dt ile) -- her
    // Step() çağrısından sonra bir kez çağrılmalı. Frame sayacını artırır.
    void EndFrame(float dt);

    // O anki fizik durumunun tam kontrol noktasını kaydeder (ör. her N
    // frame'de bir). trackedBodies: kontrol noktasına dahil edilecek
    // gövdeler.
    void RecordCheckpoint(JPH::PhysicsSystem& physics,
                          const std::vector<JPH::BodyID>& trackedBodies);

    uint32_t FrameCount() const { return frameCount; }
    const std::vector<RecordedImpulse>& Impulses() const { return impulses; }
    const std::vector<float>& FrameDeltaTimes() const { return frameDts; }
    const std::vector<ReplayCheckpoint>& Checkpoints() const { return checkpoints; }

    // Kaydı ikili dosyaya yazar. Başarısızsa false.
    bool SaveToFile(const std::string& path) const;

private:
    uint32_t frameCount = 0;
    std::vector<RecordedImpulse> impulses;
    std::vector<float> frameDts;
    std::vector<ReplayCheckpoint> checkpoints;
};

class PHYSICS_API ReplayPlayback {
public:
    // path'ten bir kaydı okur. Başarısızsa false, dahili durum değişmez.
    bool LoadFromFile(const std::string& path);

    uint32_t FrameCount() const { return frameCount; }
    // frameIndex kayıt sınırları dışındaysa 0 döner.
    float FrameDeltaTime(uint32_t frameIndex) const;

    // frameIndex'e eşit ya da ondan küçük en yakın checkpoint'i physics'e
    // geri yükler (gövde konum/rotasyon/hızlarını ayarlar) ve o
    // checkpoint'in frame indeksini döner. Uygun checkpoint yoksa 0 döner
    // (çağıran baştan başlamalı, hiçbir gövde değiştirilmez).
    uint32_t RestoreNearestCheckpoint(JPH::PhysicsSystem& physics, uint32_t frameIndex) const;

    // frameIndex'te kaydedilmiş tüm impulse'ları physics'e uygular --
    // kayıt sırasındaki davranışı eşlemek için o frame'in Step()'inden
    // ÖNCE çağrılmalı.
    void ApplyFrame(JPH::PhysicsSystem& physics, uint32_t frameIndex) const;

private:
    uint32_t frameCount = 0;
    std::vector<RecordedImpulse> impulses;
    std::vector<float> frameDts;
    std::vector<ReplayCheckpoint> checkpoints;
};

} // namespace alazforge
