#include "vehicle/WreckPersistence.h"

namespace alazforge {

WreckPersistenceSystem::WreckPersistenceSystem(ChunkGrid inGrid, float loadRadius,
                                               const std::string& savePath)
    : grid(inGrid), stream(inGrid, loadRadius, savePath) {}

void WreckPersistenceSystem::OnPlayerMove(float worldX, float worldZ) {
    stream.OnPlayerMove(worldX, worldZ);
}

void WreckPersistenceSystem::Flush() { stream.FlushDirtyChunks(); }

void WreckPersistenceSystem::AddWreck(const WreckRecord& wreck) {
    WreckChunkData& chunk = stream.GetChunkAt(wreck.position.x, wreck.position.z);
    chunk.wrecks.push_back(wreck);
    stream.MarkDirty(grid.GetChunkCoord(wreck.position.x, wreck.position.z));
}

bool WreckPersistenceSystem::RemoveWreck(uint64_t id, float worldX, float worldZ) {
    WreckChunkData& chunk = stream.GetChunkAt(worldX, worldZ);
    auto& v = chunk.wrecks;
    for (size_t i = 0; i < v.size(); ++i) {
        if (v[i].id == id) {
            v[i] = v.back();
            v.pop_back();
            stream.MarkDirty(grid.GetChunkCoord(worldX, worldZ));
            return true;
        }
    }
    return false;
}

const std::vector<WreckRecord>& WreckPersistenceSystem::GetWrecksAt(float worldX, float worldZ) {
    return stream.GetChunkAt(worldX, worldZ).wrecks;
}

size_t WreckPersistenceSystem::LoadedWreckCount(float worldX, float worldZ) {
    return stream.GetChunkAt(worldX, worldZ).wrecks.size();
}

ChunkStreamSystem<WreckChunkData>& WreckPersistenceSystem::Stream() { return stream; }

const ChunkGrid& WreckPersistenceSystem::Grid() const { return grid; }

} // namespace alazforge
