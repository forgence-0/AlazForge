#pragma once
// Su hacmi tanımı (Faz B.4). Basit eksene hizalı kutu bölge + düz su yüzeyi.

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
};

} // namespace alazforge
