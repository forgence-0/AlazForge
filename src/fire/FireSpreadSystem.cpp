#include "fire/FireSpreadSystem.h"

#include <cmath>

namespace alazforge {

namespace {
float DistanceSq(const Vec3& a, const Vec3& b) {
    const float dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
    return dx * dx + dy * dy + dz * dz;
}
} // namespace

uint32_t FireSpreadSystem::RegisterNode(const FireNodeConfig& config) {
    const uint32_t id = nextId++;
    NodeRuntime rt;
    rt.config = config;
    nodes[id] = rt;
    return id;
}

void FireSpreadSystem::RemoveNode(uint32_t id) { nodes.erase(id); }

void FireSpreadSystem::Ignite(uint32_t id) {
    auto it = nodes.find(id);
    if (it == nodes.end()) return;
    NodeRuntime& rt = it->second;
    if (!rt.config.flammable || rt.state != FireState::Unburnt) return;
    rt.state = FireState::Burning;
    rt.burnElapsedSec = 0.0f;
}

void FireSpreadSystem::Update(float dt, std::vector<uint32_t>* outIgnited,
                              std::vector<uint32_t>* outExtinguished) {
    if (dt <= 0.0f) return;

    // Yayilim: her yanan dugum, yaricapindaki her sonmemis yanabilir
    // duguma dt kadar maruziyet ekler. Once maruziyeti biriktir, SONRA
    // esigi asanlari tutustur -- boylece bu adimda yeni tutusan bir dugum
    // AYNI adimda baskalarina henuz yayilmaz (bir sonraki Update'te yayilir,
    // deterministik ve kare-sirasindan bagimsiz).
    for (auto& [unburntId, unburnt] : nodes) {
        if (unburnt.state != FireState::Unburnt || !unburnt.config.flammable) continue;
        for (const auto& [burningId, burning] : nodes) {
            if (burning.state != FireState::Burning) continue;
            const float r = burning.config.spreadRadius;
            if (DistanceSq(unburnt.config.position, burning.config.position) <= r * r)
                unburnt.exposureSec += dt;
        }
    }

    for (auto& [id, rt] : nodes) {
        if (rt.state == FireState::Unburnt && rt.config.flammable &&
            rt.exposureSec >= rt.config.igniteThresholdSec) {
            rt.state = FireState::Burning;
            rt.burnElapsedSec = 0.0f;
            if (outIgnited) outIgnited->push_back(id);
        } else if (rt.state == FireState::Burning) {
            rt.burnElapsedSec += dt;
            if (rt.burnElapsedSec >= rt.config.burnDurationSec) {
                rt.state = FireState::Burnt;
                if (outExtinguished) outExtinguished->push_back(id);
            }
        }
    }
}

FireState FireSpreadSystem::GetState(uint32_t id) const {
    auto it = nodes.find(id);
    return it != nodes.end() ? it->second.state : FireState::Unburnt;
}

size_t FireSpreadSystem::BurningCount() const {
    size_t count = 0;
    for (const auto& [id, rt] : nodes)
        if (rt.state == FireState::Burning) ++count;
    return count;
}

} // namespace alazforge
