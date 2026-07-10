#pragma once
// Malzeme veritabanı (Faz 4): balistik ve kırılma davranışını yöneten
// fiziksel malzeme özellikleri. Motor gerçekçi fiziksel varsayılanlar sağlar;
// oyun tarafı kendi malzemelerini kaydedebilir veya mevcutları ezebilir.
//
// Jolt entegrasyonu: bir body'nin malzemesi, body user data alanına
// MaterialId yazılarak belirlenir (BodyInterface::SetUserData).

#include <physics/export.h>

#include <Jolt/Jolt.h>

#include <Jolt/Physics/Body/BodyID.h>

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace JPH {
class BodyInterface;
} // namespace JPH

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

    // Temas davranışı (Jolt gövde parametreleriyle bire bir): buz kayar
    // (düşük friction), kauçuk zıplar (yüksek restitution), beton sürter.
    // ApplyToBody ile gövdeye uygulanır — tüm sistemlerde tutarlı malzeme.
    float friction = 0.5f;    // 0 = sürtünmesiz (buz), ~1 = kauçuk/asfalt
    float restitution = 0.0f; // 0 = hiç sekmez, 1 = tam elastik
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

    // Malzemenin temas davranışını (friction/restitution) gövdeye uygular
    // ve MaterialId'yi user data'ya yazar (balistik penetrasyonun beklediği
    // mevcut sözleşme). Gövde kurulduktan sonra bir kez çağrılır.
    void ApplyToBody(JPH::BodyInterface& bodyInterface, JPH::BodyID body, MaterialId id) const;

private:
    std::vector<MaterialProperties> materials;
    std::unordered_map<std::string, MaterialId> nameToId;
};

} // namespace alazforge
