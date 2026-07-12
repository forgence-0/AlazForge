#pragma once
// Patlama/blast sistemi: bir noktadan yarıçap içindeki tüm dinamik gövdelere
// mesafeye göre azalan (linear falloff) radyal impulse uygular ve etkilenen
// gövdelerin listesini döner. Hasar hesabı bilinçli olarak çağırana bırakılır:
// dönen listedeki gövde + hesaplanan şiddet ile DestructibleStructureSystem::
// ApplyDamageRadius, ragdoll aktivasyonu veya can sistemi çağrılabilir —
// böylece weapons.dll diğer yardımcı DLL'lere bağımlılık edinmez (mimari
// kural: yardımcılar yalnızca physics'e bağımlıdır).
//
// Fizik modeli: impulse = yön * baseImpulse * (1 - mesafe/yarıçap).
// Impulse gövdenin kütle merkezine değil, patlama noktasına en yakın kabuk
// noktasına değil, basitçe kütle merkezine uygulanır (oyun ölçeğinde yeterli
// ve deterministik; nokta-uygulamalı tork isteyen çağıran AddImpulse'ı kendi
// çağırabilir).

#include "core/JoltAdapter.h"

#include <weapons/export.h>

#include <Jolt/Jolt.h>

#include <Jolt/Physics/PhysicsSystem.h>

#include <vector>

namespace alazforge {

// Şiddetin mesafeyle azalma modeli
enum class ExplosionFalloff : uint8_t {
    Linear,       // 1 - d/R (oyun dengesi için basit model)
    InverseSquare // (1 - d/R)^2 — blast basıncının hızlı düşüşüne yakın,
                  // kenarda çok zayıf, merkezde yoğun (gerçekçi his)
};

struct ExplosionConfig {
    float radius = 10.0f;          // etki yarıçapı (m)
    float baseImpulseNs = 5000.0f; // merkezdeki impulse büyüklüğü (N*s)
    float upwardBias = 0.3f;       // impulse yönüne eklenen yukarı bileşen
                                   // (0=saf radyal; sinema patlaması hissi için >0)
    ExplosionFalloff falloff = ExplosionFalloff::InverseSquare;

    // Görüş hattı (siper) kontrolü: merkezden gövdeye raycast atılır; arada
    // BAŞKA bir gövde varsa hedef siperdedir — impulse/hasar almaz (duvarın
    // arkasına saklanmak gerçekten korur). Kapatılırsa eski davranış
    // (duvarlardan geçen patlama) elde edilir.
    bool losOcclusion = true;
};

struct ExplosionHit {
    JPH::BodyID body;
    float distance;        // patlama merkezine uzaklık (m)
    float falloff;         // 1 (merkez) .. 0 (yarıçap kenarı) — hasar hesabına girdi
    Vec3 impulse;          // uygulanan impulse (N*s)
    bool occluded = false; // siperdeydi: impulse uygulanmadı, falloff=0
};

class WEAPONS_API ExplosionSystem {
public:
    // Patlamayı uygular; etkilenen dinamik gövdeler outHits'e yazılır
    // (önce temizlenir). Statik gövdeler impulse almaz ama çağıran hasar
    // verebilsin diye listeye distance/falloff ile eklenir (impulse=0).
    static void Detonate(JPH::PhysicsSystem& physics, const Vec3& center,
                         const ExplosionConfig& config, std::vector<ExplosionHit>& outHits);
};

} // namespace alazforge
