#pragma once
// Malzeme veritabanı (Faz 4): balistik ve kırılma davranışını yöneten
// fiziksel malzeme özellikleri. Motor gerçekçi fiziksel varsayılanlar sağlar;
// oyun tarafı kendi malzemelerini kaydedebilir veya mevcutları ezebilir.
//
// Jolt entegrasyonu: bir body'nin malzemesi, body user data alanına
// MaterialId yazılarak belirlenir (BodyInterface::SetUserData).

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace alazforge {

using MaterialId = uint32_t;
constexpr MaterialId kInvalidMaterial = 0xFFFFFFFF;

enum class SurfaceType : uint8_t {
    Metal,
    Concrete,
    Wood,
    Soil,
    Glass,
    Flesh,
    Custom
};

struct MaterialProperties {
    std::string name;
    SurfaceType surface = SurfaceType::Custom;

    float density = 1000.0f;          // kg/m3
    float hardnessBHN = 100.0f;       // Brinell sertliği

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

class MaterialDatabase {
public:
    MaterialId Register(const MaterialProperties& props) {
        const MaterialId id = static_cast<MaterialId>(materials.size());
        nameToId[props.name] = id;
        materials.push_back(props);
        return id;
    }

    const MaterialProperties& Get(MaterialId id) const { return materials.at(id); }

    MaterialId FindByName(const std::string& name) const {
        auto it = nameToId.find(name);
        return it != nameToId.end() ? it->second : kInvalidMaterial;
    }

    size_t Count() const { return materials.size(); }

    // Gerçek dünya değerleriyle temel malzeme seti.
    // RHA eşdeğerleri yaklaşık alan-etki değerleridir (küçük kalibre için).
    static MaterialDatabase CreateDefault() {
        MaterialDatabase db;
        db.Register({"steel",    SurfaceType::Metal,    7850.0f, 150.0f, 8.0f,  17.0f, 0.05f});
        db.Register({"concrete", SurfaceType::Concrete, 2400.0f,  40.0f, 1.2f,  12.0f, 0.60f});
        db.Register({"wood",     SurfaceType::Wood,      600.0f,   3.0f, 0.15f,  0.0f, 0.30f});
        db.Register({"soil",     SurfaceType::Soil,     1600.0f,   1.0f, 0.10f,  0.0f, 0.00f});
        db.Register({"glass",    SurfaceType::Glass,    2500.0f, 500.0f, 0.40f,  0.0f, 0.95f});
        db.Register({"flesh",    SurfaceType::Flesh,    1050.0f,   1.0f, 0.07f,  0.0f, 0.00f});
        return db;
    }

private:
    std::vector<MaterialProperties> materials;
    std::unordered_map<std::string, MaterialId> nameToId;
};

} // namespace alazforge
