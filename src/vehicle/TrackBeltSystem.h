#pragma once
// Tırtıl/kayış (track/chain) parça yerleşimi -- GENEL bir geometri yardımcısı,
// tek bir araca özel değil: aynı yarıçaplı N tekerlek/makara verilince
// (kapalı döngü halinde) kayışın dış çevresi boyunca eşit aralıklı link
// (zincir parçası) transform'larını hesaplar. Tank tırtılı, paletli araç,
// konveyör bant gibi "N makara etrafında sarılı kapalı kayış" görselleştiren
// her senaryoda kullanılabilir -- VehicleSystem'e bağımlı değildir, yalnızca
// tekerlek/makara MERKEZ pozisyonlarını ve ortak yarıçapı bilir.
//
// Kısıt (bilinçli basitleştirme): tüm makaralar AYNI yarıçapta olmalı ve
// tek bir düzlemde (varsayılan: planeNormal'e dik düzlem, yani planeNormal
// aracın yanal ekseniyse kayış ileri/yukarı düzleminde) yer almalı --
// gerçek tank tırtılları da böyle çalışır (yanal simetri, tek düzlemli iz).
// Farklı yarıçaplı makaralar (örn. gergi tekerleği daha küçük) desteklenmez;
// gerekirse çağıran tüm makaraları en büyük yarıçapa göre modelleyebilir.
//
// Fizik değil, saf geometri: hiçbir Jolt gövdesi/kısıtı oluşturmaz. Kayışın
// FİZİKSEL simülasyonu istenirse (kayış gerçekten kuvvet taşısın) bunun
// yerine rope.dll'deki segment-zinciri deseni kullanılmalı -- bu sınıf
// yalnızca RENDER için "link'ler nerede duruyor" sorusuna cevap verir
// (VehicleSystem::GetWheelTransform ile aynı amaç, ama tekil tekerlek yerine
// aralarındaki kayış için).

#include "core/AlazMath.h"

#include <physics/export.h>

#include <vector>

namespace alazforge {

struct TrackBeltConfig {
    // Makara merkezleri, sıralı (kapalı döngü -- son makaradan ilkine de
    // kayış sarılır). En az 2 makara gerekir.
    std::vector<Vec3> wheelLocalPositions;
    float wheelRadius = 0.4f; // tüm makaralar bu yarıçapta varsayılır
    float linkLength = 0.2f;  // her link'in kapladığı kayış uzunluğu (m)
    // Kayışın "yanal" ekseni -- düzlem bu eksene dik. Varsayılan +x (araç
    // sağ ekseni): kayış y-z (yukarı-ileri) düzleminde yer alır, tipik bir
    // tank tırtılı gibi.
    Vec3 planeNormal{1, 0, 0};
};

class PHYSICS_API TrackBeltSystem {
public:
    explicit TrackBeltSystem(const TrackBeltConfig& config);

    // Kayışın toplam çevre uzunluğu (düz teğet parçalar + makara başına
    // sarılan yay parçaları toplamı).
    float TotalLength() const { return totalLength; }

    // linkLength'e göre kayışa sığan link sayısı (TotalLength / linkLength,
    // aşağı yuvarlanır).
    int LinkCount() const;

    // beltOffset: kayış üzerinde 0..TotalLength() arası ilerleme (m) --
    // makaraların dönüş hızından türetilir (çağıran biriktirir, örn.
    // ortalama tekerlek açısal hızı * yarıçap * dt). outLinks'e LinkCount()
    // kadar, eşit aralıklı, kayış teğetine hizalı gövde-yerel transform
    // yazılır (rotation'ın +z'si kayış ilerleme yönünü gösterir).
    void ComputeLinkTransforms(float beltOffset, std::vector<Transform>& outLinks) const;

private:
    struct PathSegment {
        bool isArc;
        // Düz parça: start/end (2B, düzlem içi). Yay parça: merkez, yarıçap,
        // başlangıç/bitiş açısı (radyan, düzlem içi).
        float startX = 0, startY = 0, endX = 0, endY = 0;
        float centerX = 0, centerY = 0, radius = 0, startAngle = 0, endAngle = 0;
        float length = 0;
    };

    TrackBeltConfig config;
    Vec3 planeU, planeV; // düzlem taban vektörleri (dünya/gövde-yerel)
    std::vector<PathSegment> segments;
    float totalLength = 0.0f;

    // 2B düzlem koordinatını (u,v tabanında) gövde-yerel Vec3'e çevirir.
    Vec3 ToLocal(float x, float y) const;
};

} // namespace alazforge
