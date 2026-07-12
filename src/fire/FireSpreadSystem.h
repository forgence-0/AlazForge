#pragma once
// Ateş/yanma yayılım sistemi: konumlandırılmış "yanabilir" düğümler
// arasında zaman-biriktirmeli, deterministik (RNG yok) yayılım simülasyonu.
// Jolt'a bağımlı değildir (fizik gövdesi gerekmez) — yalnızca konum +
// yanma parametreleriyle çalışan saf bir graf/zaman simülasyonudur; oyun
// tarafı düğümleri destructible parçalarına, terrain noktalarına veya
// tamamen ayrı bir "yanabilir nesne" listesine eşleyebilir.
//
// Yayılım kuralı: yanan bir düğüm, spreadRadius'u içindeki her sönmemiş
// yanabilir düğüme her Update(dt) çağrısında dt kadar "maruziyet" ekler;
// bir düğümün toplam maruziyeti kendi igniteThresholdSec'ini geçince
// tutuşur. Olasılıksal değil — aynı başlangıç durumu + aynı dt dizisi her
// zaman aynı yayılım sırasını üretir (replay/lockstep güvenli).

#include "core/AlazMath.h"

#include <fire/export.h>

#include <cstdint>
#include <unordered_map>
#include <vector>

namespace alazforge {

enum class FireState : uint8_t { Unburnt, Burning, Burnt };

struct FireNodeConfig {
    Vec3 position{0, 0, 0};
    float burnDurationSec = 15.0f;   // tutuştuktan ne kadar sonra söner
    float igniteThresholdSec = 2.0f; // toplam maruziyet bu süreyi geçince tutuşur
    float spreadRadius = 3.0f;       // bu düğüm YANARKEN bu yarıçaptaki diğerlerini etkiler
    bool flammable = true;           // false ise hiç tutuşmaz (taş/metal gibi engel)
};

class FIRE_API FireSpreadSystem {
public:
    // Düğümü kaydeder, kararlı bir id döner (RemoveNode sonrası diğer
    // id'ler geçerliliğini korur).
    uint32_t RegisterNode(const FireNodeConfig& config);
    void RemoveNode(uint32_t id);

    // Dışarıdan tutuşturur (patlama, kibrit, yanan bir komşudan sıçrama).
    // flammable=false veya zaten Burnt ise no-op.
    void Ignite(uint32_t id);

    // dt kadar ilerletir: yanan düğümler yakındaki yanabilir düğümlere
    // maruziyet biriktirir, eşiği aşanlar tutuşur; süresi dolan düğümler
    // söner (Burnt olur, bir daha tutuşmaz). outIgnited/outExtinguished
    // verilirse bu adımda değişen düğüm id'leri eklenir (önce temizlenmez).
    void Update(float dt, std::vector<uint32_t>* outIgnited = nullptr,
                std::vector<uint32_t>* outExtinguished = nullptr);

    FireState GetState(uint32_t id) const;
    size_t BurningCount() const;

private:
    struct NodeRuntime {
        FireNodeConfig config;
        FireState state = FireState::Unburnt;
        float exposureSec = 0.0f;    // yalnizca Unburnt iken birikir
        float burnElapsedSec = 0.0f; // yalnizca Burning iken birikir
    };

    std::unordered_map<uint32_t, NodeRuntime> nodes;
    uint32_t nextId = 1;
};

} // namespace alazforge
