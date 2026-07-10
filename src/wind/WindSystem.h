#pragma once
// Küresel rüzgar sistemi: yön + şiddet + deterministik türbülans.
// Tek bir rüzgar kaynağı tüm dünyayı etkiler; spor topu, halat, balistik,
// paraşüt ve kumaş aynı WindSystem'den beslenerek tutarlı bir atmosfer
// oluşturur. physics.dll içindedir — hem çekirdek sistemler (balistik) hem
// tüm yardımcı DLL'ler erişebilir.
//
// Determinizm sözleşmesi (BallisticsSystem/SpreadAccumulator ile aynı):
// RNG kullanılmaz. Türbülans, pozisyon+zamana bağlı sabit frekanslı sinüs
// toplamıyla üretilir — aynı (pozisyon, zaman) her zaman aynı rüzgarı verir
// (multiplayer lockstep ve replay güvenli).

#include "core/AlazMath.h"

#include <physics/export.h>

namespace alazforge {

struct WindConfig {
    Vec3 direction{1, 0, 0};    // esme yönü (normalize edilmemişse edilir)
    float baseSpeed = 5.0f;     // ortalama şiddet (m/s)
    float gustAmplitude = 2.0f; // türbülans genliği (m/s, 0 = sabit rüzgar)
    float gustFrequency = 0.3f; // türbülans frekansı (Hz civarı)
    float spatialScale = 0.02f; // pozisyona bağlı varyasyon ölçeği (1/m)
};

class PHYSICS_API WindSystem {
public:
    WindSystem();
    explicit WindSystem(const WindConfig& inConfig);

    void SetConfig(const WindConfig& inConfig);
    const WindConfig& Config() const { return config; }

    // Her frame dt kadar ilerletilir (türbülans fazı için)
    void Update(float dt) { time += dt; }
    float Time() const { return time; }

    // Verilen dünya noktasındaki anlık rüzgar hızı (m/s). Deterministik:
    // aynı pozisyon + aynı dahili zaman her zaman aynı sonucu verir.
    Vec3 GetWindAt(const Vec3& worldPos) const;

    // Kolaylık: rüzgarın bir gövdeye uyguladığı sürükleme kuvveti
    // F = dragCoefficient * (rüzgar - gövdeHızı). Çağıran, kuvveti
    // BodyInterface::AddForce ile uygular (sistem dünyaya dokunmaz —
    // hangi gövdenin rüzgardan etkileneceği oyun tarafının kararı).
    Vec3 ComputeDragForce(const Vec3& worldPos, const Vec3& bodyVelocity,
                          float dragCoefficient) const;

private:
    WindConfig config;
    Vec3 normalizedDir{1, 0, 0};
    float time = 0.0f;
};

} // namespace alazforge
