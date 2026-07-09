#pragma once
// Yıkılabilir yapılar (Faz B.5).
//
// TerrainDeformSystem'in sparse-chunk kalıcılık desenini (ChunkStreamSystem<T>
// üzerine kurulu) yeniden kullanır, ama veri modeli sürekli heightfield değil,
// oyun tarafından RegisterStructure ile verilen ayrık parça grafiğidir. Her
// parça kendi statik JPH::BoxShape gövdesi olarak oluşturulur (MaterialId-in-
// userdata sözleşmesi korunur — BallisticsSystem'in parça-başı penetrasyon
// hesabı değişmeden çalışmaya devam eder).
//
// Çökme kuralı: MaterialProperties::brittleness çökme eşiği olarak kullanılır.
// Bir parçanın destek komşularının (1 - brittleness) oranından azı sağlam
// kaldığında otomatik kırılır (cam/beton az destekle çöker, çelik/ahşap
// dayanır) — tamamen deterministik, RNG yok.
//
// Onaylanan karar: parça kırıldığında varsayılan olarak yalnızca event
// bildirilir (PieceBrokenEvent) ve statik gövdesi kaldırılır — hafif kalır.
// İsteyen DetachPieceAsDynamicBody ile kırılan parçayı gerçek dinamik bir
// Jolt gövdesine çevirip enkaz görseli oluşturabilir.

#include "core/JoltAdapter.h"
#include "destructible/DestructibleChunkData.h"
#include "destructible/DestructibleTypes.h"
#include "material_db/MaterialDatabase.h"

#include <physics_ext/export.h>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/PhysicsSystem.h>

#include <string>
#include <unordered_map>

namespace alazforge {

class PHYSICS_EXT_API DestructibleStructureSystem {
public:
    struct Config {
        int mapSize = 4096;
        int chunkSize = 64;
        float loadRadius = 200.0f;
        std::string savePath = "destructible";
    };

    explicit DestructibleStructureSystem(const Config& inConfig);

    void OnPlayerMove(float worldX, float worldZ);
    void Flush();

    // Yapıyı kaydeder: her parça için ayrı bir statik Jolt gövdesi oluşturur.
    // materials, RegisterStructure sırasında her parçanın brittleness'ını
    // önceden çözüp saklamak için kullanılır (çökme kararı zaman-kritik
    // ApplyDamage çağrısında tekrar veritabanı sorgusu yapmasın diye).
    uint64_t RegisterStructure(JPH::PhysicsSystem& physics, JPH::ObjectLayer layer,
                               const MaterialDatabase& materials, const StructureConfig& structure);

    void ApplyDamage(uint64_t structureId, uint16_t pieceIndex, float damage,
                     std::vector<PieceBrokenEvent>& outEvents);

    void ApplyDamageRadius(uint64_t structureId, const Vec3& worldPoint, float damage, float radius,
                           std::vector<PieceBrokenEvent>& outEvents);

    float GetPieceHealth(uint64_t structureId, uint16_t pieceIndex) const;
    bool IsPieceBroken(uint64_t structureId, uint16_t pieceIndex) const;

    // BallisticsSystem'in vurduğu body'yi (structureId, pieceIndex)'e çözer.
    bool FindPieceByBodyID(JPH::BodyID hitBody, uint64_t& outStructureId,
                           uint16_t& outPieceIndex) const;

    // Kırılmış bir parçayı gerçek dinamik bir Jolt gövdesine çevirir (enkaz
    // görseli için). Yalnızca zaten kırılmış parçalarda çalışır; geçersiz
    // BodyID döner (IsInvalid()) eğer parça kırılmamışsa veya bulunamazsa.
    JPH::BodyID DetachPieceAsDynamicBody(JPH::PhysicsSystem& physics, uint64_t structureId,
                                         uint16_t pieceIndex);

private:
    struct PieceRuntime {
        Vec3 localCenter;
        Vec3 halfExtents;
        float health = 0.0f;
        bool broken = false;
        MaterialId material = kInvalidMaterial;
        float brittleness = 0.0f;
        std::vector<uint16_t> supportNeighbors;
        std::vector<uint16_t> dependents; // ters bağımlılık: bu parçaya dayanan parçalar
        JPH::BodyID bodyId;
    };

    struct StructureRuntime {
        Vec3 worldOrigin;
        JPH::ObjectLayer layer = 0;
        std::vector<PieceRuntime> pieces;
    };

    Config config;
    ChunkGrid grid;
    ChunkStreamSystem<DestructibleChunkData> stream;
    JPH::PhysicsSystem* physics = nullptr;

    std::unordered_map<uint64_t, StructureRuntime> structures;
    std::unordered_map<uint32_t, std::pair<uint64_t, uint16_t>> bodyIndexToPiece;

    void ApplyDamageToPiece(uint64_t structureId, StructureRuntime& s, uint16_t pieceIndex,
                            float damage, std::vector<PieceBrokenEvent>& outEvents);
    void BreakPiece(uint64_t structureId, StructureRuntime& s, uint16_t pieceIndex,
                    std::vector<PieceBrokenEvent>& outEvents);
    Vec3 PieceWorldPosition(const StructureRuntime& s, const PieceRuntime& p) const;
};

} // namespace alazforge
