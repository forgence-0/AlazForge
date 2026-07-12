#include "scene/ReplaySystem.h"

#include "core/JoltAdapter.h"

#include <Jolt/Physics/Body/BodyLock.h>

#include <algorithm>
#include <fstream>

namespace alazforge {

namespace {

// Küçük, sürüm etiketli ikili format -- MiniJson kasıtlı olarak
// kullanılmadı: bu kayıtlar (özellikle checkpoint'ler) frame başına
// büyüyebilir, metin formatı gereksiz büyük/yavaş olurdu.
constexpr uint32_t kMagic = 0x414C525A; // "ALRZ"
constexpr uint32_t kVersion = 1;

void WriteU32(std::ofstream& out, uint32_t v) { out.write(reinterpret_cast<const char*>(&v), 4); }
void WriteF32(std::ofstream& out, float v) { out.write(reinterpret_cast<const char*>(&v), 4); }

bool ReadU32(std::ifstream& in, uint32_t& v) {
    in.read(reinterpret_cast<char*>(&v), 4);
    return static_cast<bool>(in);
}
bool ReadF32(std::ifstream& in, float& v) {
    in.read(reinterpret_cast<char*>(&v), 4);
    return static_cast<bool>(in);
}

void WriteBodyState(std::ofstream& out, const RecordedBodyState& b) {
    WriteU32(out, b.body.GetIndexAndSequenceNumber());
    WriteF32(out, b.position.x);
    WriteF32(out, b.position.y);
    WriteF32(out, b.position.z);
    WriteF32(out, b.rotation.x);
    WriteF32(out, b.rotation.y);
    WriteF32(out, b.rotation.z);
    WriteF32(out, b.rotation.w);
    WriteF32(out, b.linearVelocity.x);
    WriteF32(out, b.linearVelocity.y);
    WriteF32(out, b.linearVelocity.z);
    WriteF32(out, b.angularVelocity.x);
    WriteF32(out, b.angularVelocity.y);
    WriteF32(out, b.angularVelocity.z);
}

bool ReadBodyState(std::ifstream& in, RecordedBodyState& b) {
    uint32_t rawId = 0;
    if (!ReadU32(in, rawId)) return false;
    b.body = JPH::BodyID(rawId);
    return ReadF32(in, b.position.x) && ReadF32(in, b.position.y) && ReadF32(in, b.position.z) &&
           ReadF32(in, b.rotation.x) && ReadF32(in, b.rotation.y) && ReadF32(in, b.rotation.z) &&
           ReadF32(in, b.rotation.w) && ReadF32(in, b.linearVelocity.x) &&
           ReadF32(in, b.linearVelocity.y) && ReadF32(in, b.linearVelocity.z) &&
           ReadF32(in, b.angularVelocity.x) && ReadF32(in, b.angularVelocity.y) &&
           ReadF32(in, b.angularVelocity.z);
}

} // namespace

void ReplayRecorder::RecordImpulse(JPH::BodyID body, const Vec3& impulse) {
    impulses.push_back({frameCount, body, impulse});
}

void ReplayRecorder::EndFrame(float dt) {
    frameDts.push_back(dt);
    ++frameCount;
}

void ReplayRecorder::RecordCheckpoint(JPH::PhysicsSystem& physics,
                                      const std::vector<JPH::BodyID>& trackedBodies) {
    ReplayCheckpoint cp;
    cp.frameIndex = frameCount;
    cp.bodies.reserve(trackedBodies.size());
    for (JPH::BodyID id : trackedBodies) {
        JPH::BodyLockRead lock(physics.GetBodyLockInterface(), id);
        if (!lock.Succeeded()) continue;
        const JPH::Body& body = lock.GetBody();
        RecordedBodyState state;
        state.body = id;
        state.position = FromJolt(JPH::Vec3(body.GetPosition()));
        state.rotation = FromJolt(body.GetRotation());
        state.linearVelocity = FromJolt(body.GetLinearVelocity());
        state.angularVelocity = FromJolt(body.GetAngularVelocity());
        cp.bodies.push_back(state);
    }
    checkpoints.push_back(std::move(cp));
}

bool ReplayRecorder::SaveToFile(const std::string& path) const {
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    if (!out.is_open()) return false;

    WriteU32(out, kMagic);
    WriteU32(out, kVersion);
    WriteU32(out, frameCount);

    WriteU32(out, static_cast<uint32_t>(frameDts.size()));
    for (float dt : frameDts)
        WriteF32(out, dt);

    WriteU32(out, static_cast<uint32_t>(impulses.size()));
    for (const RecordedImpulse& imp : impulses) {
        WriteU32(out, imp.frameIndex);
        WriteU32(out, imp.body.GetIndexAndSequenceNumber());
        WriteF32(out, imp.impulse.x);
        WriteF32(out, imp.impulse.y);
        WriteF32(out, imp.impulse.z);
    }

    WriteU32(out, static_cast<uint32_t>(checkpoints.size()));
    for (const ReplayCheckpoint& cp : checkpoints) {
        WriteU32(out, cp.frameIndex);
        WriteU32(out, static_cast<uint32_t>(cp.bodies.size()));
        for (const RecordedBodyState& b : cp.bodies)
            WriteBodyState(out, b);
    }

    return out.good();
}

bool ReplayPlayback::LoadFromFile(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in.is_open()) return false;

    uint32_t magic = 0, version = 0, loadedFrameCount = 0;
    if (!ReadU32(in, magic) || magic != kMagic) return false;
    if (!ReadU32(in, version) || version != kVersion) return false;
    if (!ReadU32(in, loadedFrameCount)) return false;

    uint32_t dtCount = 0;
    if (!ReadU32(in, dtCount)) return false;
    std::vector<float> loadedDts(dtCount);
    for (uint32_t i = 0; i < dtCount; ++i)
        if (!ReadF32(in, loadedDts[i])) return false;

    uint32_t impulseCount = 0;
    if (!ReadU32(in, impulseCount)) return false;
    std::vector<RecordedImpulse> loadedImpulses(impulseCount);
    for (uint32_t i = 0; i < impulseCount; ++i) {
        uint32_t rawId = 0;
        if (!ReadU32(in, loadedImpulses[i].frameIndex) || !ReadU32(in, rawId) ||
            !ReadF32(in, loadedImpulses[i].impulse.x) ||
            !ReadF32(in, loadedImpulses[i].impulse.y) || !ReadF32(in, loadedImpulses[i].impulse.z))
            return false;
        loadedImpulses[i].body = JPH::BodyID(rawId);
    }

    uint32_t checkpointCount = 0;
    if (!ReadU32(in, checkpointCount)) return false;
    std::vector<ReplayCheckpoint> loadedCheckpoints(checkpointCount);
    for (uint32_t i = 0; i < checkpointCount; ++i) {
        uint32_t bodyCount = 0;
        if (!ReadU32(in, loadedCheckpoints[i].frameIndex) || !ReadU32(in, bodyCount)) return false;
        loadedCheckpoints[i].bodies.resize(bodyCount);
        for (uint32_t j = 0; j < bodyCount; ++j)
            if (!ReadBodyState(in, loadedCheckpoints[i].bodies[j])) return false;
    }

    frameCount = loadedFrameCount;
    frameDts = std::move(loadedDts);
    impulses = std::move(loadedImpulses);
    checkpoints = std::move(loadedCheckpoints);
    return true;
}

float ReplayPlayback::FrameDeltaTime(uint32_t frameIndex) const {
    return frameIndex < frameDts.size() ? frameDts[frameIndex] : 0.0f;
}

uint32_t ReplayPlayback::RestoreNearestCheckpoint(JPH::PhysicsSystem& physics,
                                                  uint32_t frameIndex) const {
    const ReplayCheckpoint* best = nullptr;
    for (const ReplayCheckpoint& cp : checkpoints) {
        if (cp.frameIndex <= frameIndex && (!best || cp.frameIndex > best->frameIndex)) best = &cp;
    }
    if (!best) return 0;

    JPH::BodyInterface& bi = physics.GetBodyInterface();
    for (const RecordedBodyState& b : best->bodies) {
        bi.SetPositionAndRotation(b.body, ToJoltR(b.position), ToJolt(b.rotation),
                                  JPH::EActivation::Activate);
        bi.SetLinearVelocity(b.body, ToJolt(b.linearVelocity));
        bi.SetAngularVelocity(b.body, ToJolt(b.angularVelocity));
    }
    return best->frameIndex;
}

void ReplayPlayback::ApplyFrame(JPH::PhysicsSystem& physics, uint32_t frameIndex) const {
    JPH::BodyInterface& bi = physics.GetBodyInterface();
    for (const RecordedImpulse& imp : impulses) {
        if (imp.frameIndex == frameIndex) bi.AddImpulse(imp.body, ToJolt(imp.impulse));
    }
}

} // namespace alazforge
