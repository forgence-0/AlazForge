#pragma once
// Su hacmi tanımı (Faz B.4). Eksene hizalı kutu bölge; yüzey düz olabilir
// veya dalga parametreleriyle zamana bağlı iner-kalkar (yüzen cisimler
// dalgada sallanır). Dalga tamamen deterministiktir (sinüs — RNG yok).

#include "core/AlazMath.h"

namespace alazforge {

struct WaterVolume {
    float surfaceY = 0.0f;
    // Varsayılan: sınırsız (tüm dünyayı kaplayan tek bir su düzlemi)
    Vec3 boundsMin{-1e6f, -1e6f, -1e6f};
    Vec3 boundsMax{1e6f, 1e6f, 1e6f};
    // Jolt'un ApplyBuoyancyImpulse'a iletilen akışkan yoğunluğu/gövde oranı —
    // 1.0 = akışkan ve gövde eşit yoğunlukta, >1 = daha fazla yüzerlik.
    float buoyancyFactor = 1.1f;
    float linearDrag = 0.5f;
    float angularDrag = 0.5f;
    Vec3 flowVelocity{0, 0, 0};

    // ── Dalga (0 genlik = düz yüzey, eski davranış) ────────────────────
    float waveAmplitude = 0.0f;  // dalga yüksekliğinin yarısı (m)
    float waveLength = 12.0f;    // tepe-tepe mesafe (m)
    float waveSpeed = 1.5f;      // ilerleme hızı (m/s)
    Vec3 waveDirection{1, 0, 0}; // ilerleme yönü (xz düzleminde)
};

} // namespace alazforge
