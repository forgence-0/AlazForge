#include "terrain_deform/TerrainDeformSystem.h"

#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/Shape/HeightFieldShape.h>

#include <cmath>
#include <vector>

namespace alazforge {

namespace {

uint64_t ChunkKey(const ChunkCoord& c) {
    return (static_cast<uint64_t>(static_cast<uint32_t>(c.x)) << 32) |
           static_cast<uint64_t>(static_cast<uint32_t>(c.z));
}

} // namespace

TerrainDeformSystem::TerrainDeformSystem(const Config& inConfig)
    : config(inConfig),
      cellsPerSide(static_cast<int>(static_cast<float>(inConfig.chunkSize) / inConfig.cellSize)),
      grid(inConfig.mapSize, inConfig.chunkSize),
      stream(grid, inConfig.loadRadius, inConfig.savePath) {}

void TerrainDeformSystem::OnPlayerMove(float worldX, float worldZ) {
    stream.OnPlayerMove(worldX, worldZ);
}

void TerrainDeformSystem::Flush() { stream.FlushDirtyChunks(); }

int TerrainDeformSystem::CellsPerSide() const { return cellsPerSide; }

void TerrainDeformSystem::ApplyDeformation(float worldX, float worldZ, float depth) {
    DeformationChunkData& chunk = stream.GetChunkAt(worldX, worldZ);
    const uint32_t index = CellIndex(worldX, worldZ);
    float& cell = chunk.cells[index]; // yoksa 0 olarak eklenir, varsa güncellenir
    cell = std::min(cell + depth, config.maxDepth);
    stream.MarkDirty(grid.GetChunkCoord(worldX, worldZ));
}

void TerrainDeformSystem::ApplyDeformationRadius(float worldX, float worldZ, float depth,
                                                 float radius) {
    const float cs = config.cellSize;
    const float minX = worldX - radius, maxX = worldX + radius;
    const float minZ = worldZ - radius, maxZ = worldZ + radius;

    for (float z = SnapToCellCenter(minZ); z <= maxZ; z += cs) {
        for (float x = SnapToCellCenter(minX); x <= maxX; x += cs) {
            const float dx = x - worldX, dz = z - worldZ;
            const float distSq = dx * dx + dz * dz;
            if (distSq > radius * radius) continue;
            const float falloff = 1.0f - std::sqrt(distSq) / radius;
            ApplyDeformation(x, z, depth * falloff);
        }
    }
}

float TerrainDeformSystem::GetDepthAt(float worldX, float worldZ) {
    DeformationChunkData& chunk = stream.GetChunkAt(worldX, worldZ);
    auto it = chunk.cells.find(CellIndex(worldX, worldZ));
    return it != chunk.cells.end() ? it->second : 0.0f;
}

size_t TerrainDeformSystem::TouchedCellCount(float worldX, float worldZ) {
    return stream.GetChunkAt(worldX, worldZ).cells.size();
}

ChunkStreamSystem<DeformationChunkData>& TerrainDeformSystem::Stream() { return stream; }

const ChunkGrid& TerrainDeformSystem::Grid() const { return grid; }

JPH::BodyID TerrainDeformSystem::UpdateChunkCollision(JPH::PhysicsSystem& physics, float worldX,
                                                      float worldZ, float baseHeightY,
                                                      JPH::ObjectLayer layer) {
    const ChunkCoord coord = grid.GetChunkCoord(worldX, worldZ);
    const uint64_t key = ChunkKey(coord);
    JPH::BodyInterface& bi = physics.GetBodyInterface();

    // Varsa eski govdeyi kaldir (HeightFieldShape olusturulduktan sonra
    // degistirilemez -- deformasyon guncellemesi = govdeyi yeniden kurmak)
    auto existing = chunkBodies.find(key);
    if (existing != chunkBodies.end()) {
        bi.RemoveBody(existing->second);
        bi.DestroyBody(existing->second);
        chunkBodies.erase(existing);
    }

    // Deformasyon verisini yukseklik orneklerine cevir: deformasyonsuz
    // hucre = baseHeightY, cokmus hucre = baseHeightY - cokme derinligi.
    const DeformationChunkData& chunk = stream.GetChunkAt(worldX, worldZ);
    const uint32_t sampleCount = static_cast<uint32_t>(cellsPerSide);
    std::vector<float> samples(static_cast<size_t>(sampleCount) * sampleCount, baseHeightY);
    for (const auto& [index, depth] : chunk.cells) {
        if (index < samples.size()) samples[index] = baseHeightY - depth;
    }

    // HeightFieldShape yerel uzayda (0,sample,0)..(n-1,sample,n-1) tanimlidir;
    // mOffset/mScale ile hucre boyutuna olceklenir, govde chunk'in dunya
    // orijinine (kose noktasi) yerlestirilir.
    JPH::HeightFieldShapeSettings shapeSettings(samples.data(), JPH::Vec3::sZero(),
                                                JPH::Vec3(config.cellSize, 1.0f, config.cellSize),
                                                sampleCount);
    shapeSettings.SetEmbedded();

    const JPH::RVec3 chunkOrigin(static_cast<double>(coord.x * config.chunkSize), 0.0,
                                 static_cast<double>(coord.z * config.chunkSize));
    JPH::BodyCreationSettings bodySettings(shapeSettings.Create().Get(), chunkOrigin,
                                           JPH::Quat::sIdentity(), JPH::EMotionType::Static, layer);
    const JPH::BodyID body = bi.CreateAndAddBody(bodySettings, JPH::EActivation::DontActivate);
    chunkBodies[key] = body;
    return body;
}

void TerrainDeformSystem::DestroyAllChunkCollision(JPH::PhysicsSystem& physics) {
    JPH::BodyInterface& bi = physics.GetBodyInterface();
    for (const auto& [key, body] : chunkBodies) {
        bi.RemoveBody(body);
        bi.DestroyBody(body);
    }
    chunkBodies.clear();
}

uint32_t TerrainDeformSystem::CellIndex(float worldX, float worldZ) const {
    const ChunkCoord c = grid.GetChunkCoord(worldX, worldZ);
    const float localX = worldX - static_cast<float>(c.x * config.chunkSize);
    const float localZ = worldZ - static_cast<float>(c.z * config.chunkSize);
    int cx = static_cast<int>(localX / config.cellSize);
    int cz = static_cast<int>(localZ / config.cellSize);
    cx = std::clamp(cx, 0, cellsPerSide - 1);
    cz = std::clamp(cz, 0, cellsPerSide - 1);
    return static_cast<uint32_t>(cz) * static_cast<uint32_t>(cellsPerSide) +
           static_cast<uint32_t>(cx);
}

float TerrainDeformSystem::SnapToCellCenter(float world) const {
    const float cs = config.cellSize;
    return (std::floor(world / cs) + 0.5f) * cs;
}

} // namespace alazforge
