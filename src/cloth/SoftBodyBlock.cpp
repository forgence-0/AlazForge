#include "cloth/SoftBodyBlock.h"

#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/SoftBody/SoftBodyCreationSettings.h>
#include <Jolt/Physics/SoftBody/SoftBodyMotionProperties.h>
#include <Jolt/Physics/SoftBody/SoftBodySharedSettings.h>

#include <algorithm>
#include <cfloat>

namespace alazforge {

namespace {

// Bir kupu, kosegeni (000)-(111) paylasan 6 tetrahedrona boler -- standart,
// hacim-tam (gap/overlap olmayan) bir ayristirma. c[bit pattern xyz] seklinde
// 8 kose indeksi alir (bit0=x, bit1=y, bit2=z).
void AddCubeTetrahedra(JPH::SoftBodySharedSettings::Volume* volumes, uint32_t c[8],
                       float compliance) {
    // Kosegen disindaki 6 kose, kosegen ekseni etrafinda alti-genlik bir
    // dongu olusturur: 1(100)-3(110)-2(010)-6(011)-4(001)-5(101)-1
    const uint32_t ring[6] = {c[1], c[3], c[2], c[6], c[4], c[5]};
    for (int i = 0; i < 6; ++i) {
        const uint32_t a = ring[i];
        const uint32_t b = ring[(i + 1) % 6];
        volumes[i] = JPH::SoftBodySharedSettings::Volume(c[0], c[7], a, b, compliance);
    }
}

} // namespace

SoftBodyBlock::SoftBodyBlock(const SoftBodyBlockConfig& inConfig) : config(inConfig) {
    config.segmentsX = std::max(2, config.segmentsX);
    config.segmentsY = std::max(2, config.segmentsY);
    config.segmentsZ = std::max(2, config.segmentsZ);
}

SoftBodyBlock::~SoftBodyBlock() { Destroy(); }

void SoftBodyBlock::Create(JPH::PhysicsSystem& inPhysics, const Vec3& worldCenter,
                           JPH::ObjectLayer layer) {
    Destroy();
    physics = &inPhysics;

    const int sx = config.segmentsX;
    const int sy = config.segmentsY;
    const int sz = config.segmentsZ;
    const float dx = config.sizeX / static_cast<float>(sx - 1);
    const float dy = config.sizeY / static_cast<float>(sy - 1);
    const float dz = config.sizeZ / static_cast<float>(sz - 1);
    const float invMass = 1.0f / std::max(1.0e-4f, config.massPerVertex);

    auto index = [sx, sy](int x, int y, int z) {
        return static_cast<JPH::uint32>(x + sx * (y + sy * z));
    };

    JPH::Ref<JPH::SoftBodySharedSettings> settings = new JPH::SoftBodySharedSettings;
    for (int z = 0; z < sz; ++z)
        for (int y = 0; y < sy; ++y)
            for (int x = 0; x < sx; ++x) {
                JPH::SoftBodySharedSettings::Vertex v;
                v.mPosition =
                    JPH::Float3((static_cast<float>(x) - 0.5f * static_cast<float>(sx - 1)) * dx,
                                (static_cast<float>(y) - 0.5f * static_cast<float>(sy - 1)) * dy,
                                (static_cast<float>(z) - 0.5f * static_cast<float>(sz - 1)) * dz);
                v.mInvMass = invMass;
                settings->mVertices.push_back(v);
            }

    // Ic hucreler: her biri 6 tetrahedrona bolunur, her tetrahedron icin
    // GERCEK hacim koruma kisiti eklenir.
    for (int z = 0; z + 1 < sz; ++z)
        for (int y = 0; y + 1 < sy; ++y)
            for (int x = 0; x + 1 < sx; ++x) {
                uint32_t c[8] = {
                    index(x, y, z),         index(x + 1, y, z),         index(x, y + 1, z),
                    index(x + 1, y + 1, z), index(x, y, z + 1),         index(x + 1, y, z + 1),
                    index(x, y + 1, z + 1), index(x + 1, y + 1, z + 1),
                };
                JPH::SoftBodySharedSettings::Volume tets[6];
                AddCubeTetrahedra(tets, c, config.volumeCompliance);
                for (const auto& t : tets)
                    settings->mVolumeConstraints.push_back(t);
            }

    // Dis yuzey: 6 yuzun her biri icin kenar/makas kisitlarinin
    // turetilebilmesi adina ucgenler eklenir (yuzey gerginligi + collision
    // mesh'i).
    auto addQuad = [&](uint32_t a, uint32_t b, uint32_t c, uint32_t d) {
        settings->AddFace(JPH::SoftBodySharedSettings::Face(a, b, c));
        settings->AddFace(JPH::SoftBodySharedSettings::Face(a, c, d));
    };
    for (int y = 0; y + 1 < sy; ++y)
        for (int z = 0; z + 1 < sz; ++z) {
            addQuad(index(0, y, z), index(0, y, z + 1), index(0, y + 1, z + 1),
                    index(0, y + 1, z)); // -X
            addQuad(index(sx - 1, y, z + 1), index(sx - 1, y, z), index(sx - 1, y + 1, z),
                    index(sx - 1, y + 1, z + 1)); // +X
        }
    for (int x = 0; x + 1 < sx; ++x)
        for (int z = 0; z + 1 < sz; ++z) {
            addQuad(index(x, 0, z + 1), index(x, 0, z), index(x + 1, 0, z),
                    index(x + 1, 0, z + 1)); // -Y
            addQuad(index(x, sy - 1, z), index(x, sy - 1, z + 1), index(x + 1, sy - 1, z + 1),
                    index(x + 1, sy - 1, z)); // +Y
        }
    for (int x = 0; x + 1 < sx; ++x)
        for (int y = 0; y + 1 < sy; ++y) {
            addQuad(index(x, y, 0), index(x + 1, y, 0), index(x + 1, y + 1, 0),
                    index(x, y + 1, 0)); // -Z
            addQuad(index(x + 1, y, sz - 1), index(x, y, sz - 1), index(x, y + 1, sz - 1),
                    index(x + 1, y + 1, sz - 1)); // +Z
        }

    const JPH::SoftBodySharedSettings::VertexAttributes attributes(config.edgeCompliance,
                                                                   config.edgeCompliance, FLT_MAX);
    settings->CreateConstraints(&attributes, 1);
    settings->CalculateVolumeConstraintVolumes();
    settings->Optimize();

    JPH::SoftBodyCreationSettings bodySettings(settings, ToJoltR(worldCenter),
                                               JPH::Quat::sIdentity(), layer);
    body =
        physics->GetBodyInterface().CreateAndAddSoftBody(bodySettings, JPH::EActivation::Activate);
}

size_t SoftBodyBlock::GetVertexPositions(std::vector<Vec3>& out) const {
    out.clear();
    if (!physics || body.IsInvalid()) return 0;
    JPH::BodyLockRead lock(physics->GetBodyLockInterface(), body);
    if (!lock.Succeeded()) return 0;
    const JPH::Body& b = lock.GetBody();
    if (!b.IsSoftBody()) return 0;

    const auto* motion = static_cast<const JPH::SoftBodyMotionProperties*>(b.GetMotionProperties());
    const JPH::RMat44 comTransform = b.GetCenterOfMassTransform();
    out.reserve(motion->GetVertices().size());
    for (const JPH::SoftBodyVertex& v : motion->GetVertices()) {
        const JPH::RVec3 worldPos = comTransform * v.mPosition;
        out.push_back(FromJolt(JPH::Vec3(worldPos)));
    }
    return out.size();
}

float SoftBodyBlock::ComputeVolume() const {
    if (!physics || body.IsInvalid()) return 0.0f;
    JPH::BodyLockRead lock(physics->GetBodyLockInterface(), body);
    if (!lock.Succeeded()) return 0.0f;
    const JPH::Body& b = lock.GetBody();
    if (!b.IsSoftBody()) return 0.0f;

    const auto* motion = static_cast<const JPH::SoftBodyMotionProperties*>(b.GetMotionProperties());
    const auto& vertices = motion->GetVertices();
    const auto& sharedSettings = motion->GetSettings();

    float sixVolumeSum = 0.0f;
    for (const JPH::SoftBodySharedSettings::Volume& t : sharedSettings->mVolumeConstraints) {
        const JPH::Vec3 p0 = vertices[t.mVertex[0]].mPosition;
        const JPH::Vec3 p1 = vertices[t.mVertex[1]].mPosition;
        const JPH::Vec3 p2 = vertices[t.mVertex[2]].mPosition;
        const JPH::Vec3 p3 = vertices[t.mVertex[3]].mPosition;
        sixVolumeSum += (p1 - p0).Cross(p2 - p0).Dot(p3 - p0);
    }
    return std::abs(sixVolumeSum) / 6.0f;
}

void SoftBodyBlock::Destroy() {
    if (!physics) return;
    if (!body.IsInvalid()) {
        JPH::BodyInterface& bi = physics->GetBodyInterface();
        bi.RemoveBody(body);
        bi.DestroyBody(body);
        body = JPH::BodyID();
    }
    physics = nullptr;
}

} // namespace alazforge
