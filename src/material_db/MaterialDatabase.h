#pragma once
// Malzeme veritabanı (Faz 4): balistik ve kırılma davranışını yöneten
// fiziksel malzeme özellikleri. Motor gerçekçi fiziksel varsayılanlar sağlar;
// oyun tarafı kendi malzemelerini kaydedebilir veya mevcutları ezebilir.
//
// Jolt entegrasyonu: bir body'nin malzemesi, body user data alanına
// MaterialId yazılarak belirlenir (BodyInterface::SetUserData).

#include <physics/export.h>

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace alazforge {

using MaterialId = uint32_t;
constexpr MaterialId kInvalidMaterial = 0xFFFFFFFF;

enum class SurfaceType : uint8_t { Metal, Concrete, Wood, Soil, Glass, Flesh, Custom };

struct MaterialProperties {
    std::string name;
    SurfaceType surface = SurfaceType::Custom;

    float density = 1000.0f;    // kg/m3
    float hardnessBHN = 100.0f; // Brinell sertliği

    // Penetrasyon direnci: 1 cm malzemenin kaç mm RHA çeliğe denk geldiği.
    // Mermi penetrasyonu mm RHA cinsinden ifade edilir (bkz. BulletParams).
    float rhaEquivalentPerCm = 1.0f;

    // Sekme: merminin yüzeye göre sıyırma açısı (derece) bu eşiğin altındaysa
    // ve malzeme yeterince sertse mermi seker.
    float ricochetCriticalAngleDeg = 0.0f; // 0 = bu malzemeden sekme olmaz

    // Kırılma: 0 = sünek (çelik, toprak), 1 = tamamen gevrek (cam).
    // Decal/parçalanma efektlerinin seçiminde kullanılır.
    float brittleness = 0.0f;
};

class PHYSICS_API MaterialDatabase {
public:
    MaterialId Register(const MaterialProperties& props);
    const MaterialProperties& Get(MaterialId id) const;
    MaterialId FindByName(const std::string& name) const;
    size_t Count() const;

    // Gerçek dünya değerleriyle temel malzeme seti.
    // RHA eşdeğerleri yaklaşık alan-etki değerleridir (küçük kalibre için).
    static MaterialDatabase CreateDefault();

private:
    std::vector<MaterialProperties> materials;
    std::unordered_map<std::string, MaterialId> nameToId;
};

} // namespace alazforge
