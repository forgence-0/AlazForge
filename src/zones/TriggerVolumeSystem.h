#pragma once
// Tetikleyici hacim (trigger volume): eksene hizalı kutu bölgeler, izlenen
// bir gövde bölgeye girdiğinde/çıktığında bir olay üretir. Fiziksel bir
// çarpışma DEĞİLDİR — yalnızca konum sorgusudur (FrictionZoneSystem'in
// "gövde bölge içinde mi" mantığıyla aynı temel, ama etki uygulamak yerine
// yalnızca girildi/çıkıldı olayı üretir). Oyun tarafı bu olaylarla checkpoint,
// hasar alanı, ses tetikleyici, görev mantığı vb. kurabilir.
//
// FrictionZoneSystem/MagneticFieldSystem/WindSystem gibi zaten üretimde
// olan sistemleri ortak bir taban sınıfa taşımak burada BİLİNÇLİ OLARAK
// yapılmadı: üçü de farklı per-frame etki mantığına sahip (sürtünme
// değiştirme, radyal kuvvet, global ivme) ve zaten test edilmiş durumda —
// kozmetik bir birleştirme için regresyon riski almak yerine, tetikleyici
// hacim bağımsız/küçük bir sistem olarak eklendi.

#include "core/AlazMath.h"

#include <zones/export.h>

#include <Jolt/Jolt.h>

#include <Jolt/Physics/PhysicsSystem.h>

#include <cstdint>
#include <vector>

namespace alazforge {

struct TriggerVolume {
    Vec3 boundsMin{0, 0, 0};
    Vec3 boundsMax{0, 0, 0};
    // Oyun tarafının olayı hangi hacme ait olduğunu tanıyabilmesi için
    // (ör. "checkpoint_3", "damage_zone_lava") — AlazForge bu id'ye
    // hiçbir anlam yüklemez, olduğu gibi geri döner.
    uint64_t userTag = 0;
};

enum class TriggerEventType : uint8_t { Entered, Exited };

struct TriggerEvent {
    size_t volumeIndex = 0;
    uint64_t userTag = 0;
    JPH::BodyID body;
    TriggerEventType type = TriggerEventType::Entered;
};

class ZONES_API TriggerVolumeSystem {
public:
    // Hacim ekler; kararlı index döner.
    size_t AddVolume(const TriggerVolume& volume);
    void RemoveVolume(size_t index);

    // Gövdeyi izlemeye al/çıkar. İzlenmeyen gövdeler için olay üretilmez.
    void TrackBody(JPH::BodyID body);
    void UntrackBody(JPH::BodyID body);

    // İzlenen her gövdenin konumunu tüm hacimlere karşı denetler; durum
    // değişimlerini (girdi/çıktı) outEvents'e EKLER (önce temizlemez —
    // çağıran birden fazla sistemin olaylarını aynı vektörde biriktirebilir).
    void Update(JPH::PhysicsSystem& physics, std::vector<TriggerEvent>& outEvents);

    // Gövdenin şu anda hangi hacim(ler) içinde olduğunu döner.
    bool IsBodyInVolume(JPH::BodyID body, size_t volumeIndex) const;

private:
    struct StoredVolume {
        TriggerVolume volume;
        bool active = false;
    };
    struct TrackedBody {
        JPH::BodyID body;
        std::vector<bool> insideVolume; // volumeIndex -> o an içinde mi
    };

    std::vector<StoredVolume> volumes;
    std::vector<TrackedBody> tracked;
};

} // namespace alazforge
