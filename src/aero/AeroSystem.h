#pragma once
// Aerodinamik yardımcılar: paraşüt ve planör/wingsuit.
//
// Paraşüt: hıza karşı kuadratik sürükleme kuvveti (F = -c·|v_rel|·v_rel,
// v_rel = gövde hızı - rüzgar). Kuvvet, gövde merkezinin ÜSTÜNDEKİ bir
// bağlantı noktasına uygulanır — paraşütlü cisim gerçekte olduğu gibi
// kendini dikleştirir ve sarkaç gibi salınır. Rüzgar paraşütü sürükler.
//
// Planör: hücum açısına göre basitleştirilmiş taşıma (lift) + sürükleme.
// Taşıma, hava-bağıl hıza dik, kanat düzleminde yukarı yönde uygulanır —
// ileri hız irtifaya çevrilir (süzülme), rüzgara karşı süzülme mesafesi
// kısalır. Gerçek kanat profili simülasyonu değil, oyun ölçeğinde inandırıcı
// enerji dengesi hedeflenir.
//
// Her iki fonksiyon da dünyaya kuvvet uygulamaktan fazlasını yapmaz: hangi
// gövdenin paraşütlü olduğu, ne zaman açıldığı oyun tarafının kararıdır.

#include "core/JoltAdapter.h"

#include <aero/export.h>

#include <Jolt/Jolt.h>

#include <Jolt/Physics/PhysicsSystem.h>

namespace alazforge {

struct ParachuteConfig {
    float dragCoefficient = 25.0f; // c: buyuk yuvarlak parasut ~20-30 (80kg icin ~5-6 m/s terminal)
    float attachHeight = 2.0f;     // kuvvetin uygulandigi nokta: govde merkezinin ustu (m)
};

struct GliderConfig {
    float liftCoefficient = 1.2f;  // taşıma katsayısı (kanat alanı dahil, toplu)
    float dragCoefficient = 0.08f; // gövde+kanat sürüklemesi
};

class AERO_API AeroSystem {
public:
    // Paraşüt sürüklemesini gövdeye uygular (her frame, Step'ten önce).
    // wind: WindSystem::GetWindAt sonucu (rüzgar paraşütü sürükler).
    static void ApplyParachute(JPH::PhysicsSystem& physics, JPH::BodyID body,
                               const ParachuteConfig& config, const Vec3& wind);

    // Planör taşıma+sürüklemesini gövdeye uygular. forward: gövdenin ileri
    // yönü (kanat doğrultusu bundan türetilir, dünya uzayı, normalize).
    static void ApplyGlider(JPH::PhysicsSystem& physics, JPH::BodyID body,
                            const GliderConfig& config, const Vec3& forward, const Vec3& wind);
};

} // namespace alazforge
