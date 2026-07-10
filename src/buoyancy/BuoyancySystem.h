#pragma once
// Yüzerlik sistemi (Faz B.4). Jolt'un hazır JPH::Body::ApplyBuoyancyImpulse
// fonksiyonunu kullanır — sıfırdan fizik yazılmaz.
//
// NOT: Jolt submodule bu ortamda checkout edilmediği için
// ApplyBuoyancyImpulse'ın tam parametre sırası/anlamı gerçek header'lara
// karşı doğrulanamadı; Jolt'un BuoyancyTest örneğinden bilinen genel imza
// (yüzey pozisyonu/normali, buoyancy, lineer/açısal sürükleme, akış hızı,
// yerçekimi, dt) kullanıldı — ilk derlemede teyit edilmeli.

#include "buoyancy/WaterVolume.h"
#include "core/JoltAdapter.h"

#include <buoyancy/export.h>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/PhysicsSystem.h>

#include <vector>

namespace alazforge {

class BUOYANCY_API BuoyancySystem {
public:
    // Hacmi ekler, kararlı bir index döner (RemoveVolume sonrası diğer
    // index'ler geçerliliğini korur).
    size_t AddVolume(const WaterVolume& volume);
    void RemoveVolume(size_t index);

    void TrackBody(JPH::BodyID body);
    void UntrackBody(JPH::BodyID body);

    // İzlenen her gövde için, içinde bulunduğu su hacmi varsa buoyancy
    // impulse'unu uygular.
    void Update(JPH::PhysicsSystem& physics, float dt);

    bool IsSubmerged(JPH::PhysicsSystem& physics, JPH::BodyID body) const;

    // Bir dünya noktasındaki su durumu — CharacterController gibi rigid
    // body OLMAYAN aktörlerin (CharacterVirtual dünyada gövde olarak
    // yaşamaz, TrackBody ile izlenemez) yüzme mekaniği kurabilmesi için:
    // karakter tarafı her frame sorgular, submergedDepth > 0 ise kendi
    // hızına kaldırma + akıntıyı uygular.
    struct WaterState {
        bool inWater = false;
        float surfaceY = 0.0f;       // içinde bulunulan hacmin yüzey yüksekliği
        float submergedDepth = 0.0f; // nokta yüzeyin ne kadar altında (m, >=0)
        Vec3 flowVelocity{0, 0, 0};  // hacmin akıntı hızı (m/s)
    };
    WaterState QueryWaterState(const Vec3& worldPos) const;

private:
    struct StoredVolume {
        WaterVolume volume;
        bool active = false;
    };

    std::vector<StoredVolume> volumes;
    std::vector<JPH::BodyID> trackedBodies;

    const WaterVolume* FindVolumeContaining(const Vec3& worldPos) const;
};

} // namespace alazforge
