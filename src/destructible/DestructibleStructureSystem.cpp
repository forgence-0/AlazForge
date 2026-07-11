#include "destructible/DestructibleStructureSystem.h"

#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>

#include <cmath>

namespace alazforge {

DestructibleStructureSystem::DestructibleStructureSystem(const Config& inConfig)
    : config(inConfig),
      grid(inConfig.mapSize, inConfig.chunkSize),
      stream(grid, inConfig.loadRadius, inConfig.savePath) {}

void DestructibleStructureSystem::OnPlayerMove(float worldX, float worldZ) {
    stream.OnPlayerMove(worldX, worldZ);
}

void DestructibleStructureSystem::Flush() { stream.FlushDirtyChunks(); }

Vec3 DestructibleStructureSystem::PieceWorldPosition(const StructureRuntime& s,
                                                     const PieceRuntime& p) const {
    return Vec3{s.worldOrigin.x + p.localCenter.x, s.worldOrigin.y + p.localCenter.y,
                s.worldOrigin.z + p.localCenter.z};
}

uint64_t DestructibleStructureSystem::RegisterStructure(JPH::PhysicsSystem& inPhysics,
                                                        JPH::ObjectLayer layer,
                                                        const MaterialDatabase& materials,
                                                        const StructureConfig& structureConfig) {
    physics = &inPhysics;
    JPH::BodyInterface& bi = physics->GetBodyInterface();

    StructureRuntime runtime;
    runtime.worldOrigin = structureConfig.worldOrigin;
    runtime.layer = layer;
    runtime.pieces.resize(structureConfig.pieces.size());

    for (size_t i = 0; i < structureConfig.pieces.size(); ++i) {
        const PieceConfig& pc = structureConfig.pieces[i];
        PieceRuntime& pr = runtime.pieces[i];
        pr.localCenter = pc.localCenter;
        pr.halfExtents = pc.halfExtents;
        pr.health = pc.healthMax;
        pr.material = pc.material;
        pr.supportNeighbors = pc.supportNeighbors;
        pr.brittleness = (pc.material != kInvalidMaterial && pc.material < materials.Count())
                             ? materials.Get(pc.material).brittleness
                             : 0.0f;

        const Vec3 worldPos = Vec3{structureConfig.worldOrigin.x + pc.localCenter.x,
                                   structureConfig.worldOrigin.y + pc.localCenter.y,
                                   structureConfig.worldOrigin.z + pc.localCenter.z};

        JPH::BoxShapeSettings shapeSettings(ToJolt(pc.halfExtents));
        shapeSettings.SetEmbedded();
        JPH::BodyCreationSettings bodySettings(shapeSettings.Create().Get(), ToJoltR(worldPos),
                                               JPH::Quat::sIdentity(), JPH::EMotionType::Static,
                                               layer);
        bodySettings.mUserData = pc.material;
        pr.bodyId = bi.CreateAndAddBody(bodySettings, JPH::EActivation::DontActivate);

        bodyIndexToPiece[pr.bodyId.GetIndexAndSequenceNumber()] = {structureConfig.structureId,
                                                                   static_cast<uint16_t>(i)};
    }

    // Ters bağımlılık haritası: piece[i]'nin desteklerinden biri kırılırsa
    // hangi parçaların yeniden değerlendirilmesi gerektiğini bilmek için.
    for (size_t i = 0; i < runtime.pieces.size(); ++i) {
        for (uint16_t neighbor : runtime.pieces[i].supportNeighbors) {
            if (neighbor < runtime.pieces.size())
                runtime.pieces[neighbor].dependents.push_back(static_cast<uint16_t>(i));
        }
    }

    physics->OptimizeBroadPhase();
    structures[structureConfig.structureId] = std::move(runtime);
    return structureConfig.structureId;
}

void DestructibleStructureSystem::BreakPiece(uint64_t structureId, StructureRuntime& s,
                                             uint16_t pieceIndex,
                                             std::vector<PieceBrokenEvent>& outEvents) {
    PieceRuntime& p = s.pieces[pieceIndex];
    if (p.broken) return;
    p.broken = true;
    p.health = 0.0f;

    if (!p.bodyId.IsInvalid() && physics) {
        JPH::BodyInterface& bi = physics->GetBodyInterface();
        bodyIndexToPiece.erase(p.bodyId.GetIndexAndSequenceNumber());
        bi.RemoveBody(p.bodyId);
        bi.DestroyBody(p.bodyId);
        p.bodyId = JPH::BodyID();
    }

    PieceBrokenEvent ev;
    ev.structureId = structureId;
    ev.pieceIndex = pieceIndex;
    ev.worldPosition = PieceWorldPosition(s, p);
    ev.material = p.material;
    outEvents.push_back(ev);

    DestructibleChunkData& chunk = stream.GetChunkAt(ev.worldPosition.x, ev.worldPosition.z);
    chunk.records.push_back({structureId, pieceIndex, 0.0f, true});
    stream.MarkDirty(grid.GetChunkCoord(ev.worldPosition.x, ev.worldPosition.z));
}

void DestructibleStructureSystem::ApplyDamageToPiece(uint64_t structureId, StructureRuntime& s,
                                                     uint16_t pieceIndex, float damage,
                                                     std::vector<PieceBrokenEvent>& outEvents) {
    if (pieceIndex >= s.pieces.size()) return;
    PieceRuntime& piece = s.pieces[pieceIndex];
    if (piece.broken) return;

    piece.health -= damage;
    if (piece.health > 0.0f) return;

    // Kademeli çökme: bu parça kırılınca ona dayanan komşuların destek
    // oranı düşer; brittleness eşiğinin altına düşenler de kuyruğa eklenir.
    std::vector<uint16_t> queue{pieceIndex};
    size_t head = 0;
    while (head < queue.size()) {
        const uint16_t idx = queue[head++];
        if (s.pieces[idx].broken) continue;
        BreakPiece(structureId, s, idx, outEvents);

        for (uint16_t dep : s.pieces[idx].dependents) {
            PieceRuntime& depPiece = s.pieces[dep];
            if (depPiece.broken || depPiece.supportNeighbors.empty()) continue;
            size_t intact = 0;
            for (uint16_t n : depPiece.supportNeighbors)
                if (n < s.pieces.size() && !s.pieces[n].broken) ++intact;
            const float intactRatio =
                static_cast<float>(intact) / static_cast<float>(depPiece.supportNeighbors.size());
            if (intactRatio < (1.0f - depPiece.brittleness)) queue.push_back(dep);
        }
    }
}

void DestructibleStructureSystem::ApplyDamage(uint64_t structureId, uint16_t pieceIndex,
                                              float damage,
                                              std::vector<PieceBrokenEvent>& outEvents) {
    auto it = structures.find(structureId);
    if (it == structures.end()) return;
    ApplyDamageToPiece(structureId, it->second, pieceIndex, damage, outEvents);
}

void DestructibleStructureSystem::ApplyDamageRadius(uint64_t structureId, const Vec3& worldPoint,
                                                    float damage, float radius,
                                                    std::vector<PieceBrokenEvent>& outEvents) {
    auto it = structures.find(structureId);
    if (it == structures.end()) return;
    StructureRuntime& s = it->second;

    for (uint16_t i = 0; i < static_cast<uint16_t>(s.pieces.size()); ++i) {
        if (s.pieces[i].broken) continue;
        const Vec3 piecePos = PieceWorldPosition(s, s.pieces[i]);
        const float dx = piecePos.x - worldPoint.x;
        const float dy = piecePos.y - worldPoint.y;
        const float dz = piecePos.z - worldPoint.z;
        const float dist = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (dist > radius) continue;
        const float falloff = 1.0f - dist / radius;
        ApplyDamageToPiece(structureId, s, i, damage * falloff, outEvents);
    }
}

float DestructibleStructureSystem::GetPieceHealth(uint64_t structureId, uint16_t pieceIndex) const {
    auto it = structures.find(structureId);
    if (it == structures.end() || pieceIndex >= it->second.pieces.size()) return 0.0f;
    return it->second.pieces[pieceIndex].health;
}

bool DestructibleStructureSystem::IsPieceBroken(uint64_t structureId, uint16_t pieceIndex) const {
    auto it = structures.find(structureId);
    if (it == structures.end() || pieceIndex >= it->second.pieces.size()) return true;
    return it->second.pieces[pieceIndex].broken;
}

bool DestructibleStructureSystem::FindPieceByBodyID(JPH::BodyID hitBody, uint64_t& outStructureId,
                                                    uint16_t& outPieceIndex) const {
    auto it = bodyIndexToPiece.find(hitBody.GetIndexAndSequenceNumber());
    if (it == bodyIndexToPiece.end()) return false;
    outStructureId = it->second.first;
    outPieceIndex = it->second.second;
    return true;
}

JPH::BodyID DestructibleStructureSystem::DetachPieceAsDynamicBody(JPH::PhysicsSystem& physicsIn,
                                                                  uint64_t structureId,
                                                                  uint16_t pieceIndex) {
    auto it = structures.find(structureId);
    if (it == structures.end() || pieceIndex >= it->second.pieces.size()) return JPH::BodyID();
    StructureRuntime& s = it->second;
    PieceRuntime& p = s.pieces[pieceIndex];
    if (!p.broken) return JPH::BodyID(); // yalnizca zaten kirilmis parcalar donusturulebilir

    const Vec3 worldPos = PieceWorldPosition(s, p);
    JPH::BoxShapeSettings shapeSettings(ToJolt(p.halfExtents));
    shapeSettings.SetEmbedded();
    JPH::BodyCreationSettings bodySettings(shapeSettings.Create().Get(), ToJoltR(worldPos),
                                           JPH::Quat::sIdentity(), JPH::EMotionType::Dynamic,
                                           s.layer);
    bodySettings.mUserData = p.material;
    return physicsIn.GetBodyInterface().CreateAndAddBody(bodySettings, JPH::EActivation::Activate);
}

size_t DestructibleStructureSystem::DetachBrokenPieces(JPH::PhysicsSystem& physicsIn,
                                                       const std::vector<PieceBrokenEvent>& events,
                                                       std::vector<JPH::BodyID>& outBodies) {
    size_t created = 0;
    for (const PieceBrokenEvent& e : events) {
        const JPH::BodyID body = DetachPieceAsDynamicBody(physicsIn, e.structureId, e.pieceIndex);
        if (!body.IsInvalid()) {
            outBodies.push_back(body);
            ++created;
        }
    }
    return created;
}

} // namespace alazforge
