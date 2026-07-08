// Faz 1 doğrulama testi: yerçekimi altında düşen bir küp.
// Statik bir zemin + 10m yükseklikten bırakılan 1m'lik dinamik küp.
// Küpün zemine oturması ve uyku moduna geçmesi beklenir.

#include "core/JoltAdapter.h"

#include <Jolt/Jolt.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyActivationListener.h>

#include <cstdio>
#include <cmath>
#include <thread>

using namespace JPH;
using namespace JPH::literals;

// ── Katman tanımları (Jolt HelloWorld düzeni) ───────────────────────
namespace Layers {
    static constexpr ObjectLayer NON_MOVING = 0;
    static constexpr ObjectLayer MOVING = 1;
    static constexpr ObjectLayer NUM_LAYERS = 2;
}

namespace BroadPhaseLayers {
    static constexpr BroadPhaseLayer NON_MOVING(0);
    static constexpr BroadPhaseLayer MOVING(1);
    static constexpr uint NUM_LAYERS(2);
}

class BPLayerInterfaceImpl final : public BroadPhaseLayerInterface {
public:
    BPLayerInterfaceImpl() {
        mObjectToBroadPhase[Layers::NON_MOVING] = BroadPhaseLayers::NON_MOVING;
        mObjectToBroadPhase[Layers::MOVING] = BroadPhaseLayers::MOVING;
    }

    uint GetNumBroadPhaseLayers() const override { return BroadPhaseLayers::NUM_LAYERS; }

    BroadPhaseLayer GetBroadPhaseLayer(ObjectLayer inLayer) const override {
        JPH_ASSERT(inLayer < Layers::NUM_LAYERS);
        return mObjectToBroadPhase[inLayer];
    }

#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
    const char* GetBroadPhaseLayerName(BroadPhaseLayer inLayer) const override {
        switch ((BroadPhaseLayer::Type)inLayer) {
        case (BroadPhaseLayer::Type)BroadPhaseLayers::NON_MOVING: return "NON_MOVING";
        case (BroadPhaseLayer::Type)BroadPhaseLayers::MOVING: return "MOVING";
        default: JPH_ASSERT(false); return "INVALID";
        }
    }
#endif

private:
    BroadPhaseLayer mObjectToBroadPhase[Layers::NUM_LAYERS];
};

class ObjectVsBroadPhaseLayerFilterImpl : public ObjectVsBroadPhaseLayerFilter {
public:
    bool ShouldCollide(ObjectLayer inLayer1, BroadPhaseLayer inLayer2) const override {
        switch (inLayer1) {
        case Layers::NON_MOVING: return inLayer2 == BroadPhaseLayers::MOVING;
        case Layers::MOVING: return true;
        default: JPH_ASSERT(false); return false;
        }
    }
};

class ObjectLayerPairFilterImpl : public ObjectLayerPairFilter {
public:
    bool ShouldCollide(ObjectLayer inObject1, ObjectLayer inObject2) const override {
        switch (inObject1) {
        case Layers::NON_MOVING: return inObject2 == Layers::MOVING;
        case Layers::MOVING: return true;
        default: JPH_ASSERT(false); return false;
        }
    }
};

int main() {
    RegisterDefaultAllocator();
    Factory::sInstance = new Factory();
    RegisterTypes();

    TempAllocatorImpl temp_allocator(10 * 1024 * 1024);
    JobSystemThreadPool job_system(cMaxPhysicsJobs, cMaxPhysicsBarriers,
                                   (int)std::thread::hardware_concurrency() - 1);

    const uint cMaxBodies = 1024;
    const uint cNumBodyMutexes = 0;
    const uint cMaxBodyPairs = 1024;
    const uint cMaxContactConstraints = 1024;

    BPLayerInterfaceImpl broad_phase_layer_interface;
    ObjectVsBroadPhaseLayerFilterImpl object_vs_broadphase_layer_filter;
    ObjectLayerPairFilterImpl object_vs_object_layer_filter;

    PhysicsSystem physics_system;
    physics_system.Init(cMaxBodies, cNumBodyMutexes, cMaxBodyPairs, cMaxContactConstraints,
                        broad_phase_layer_interface, object_vs_broadphase_layer_filter,
                        object_vs_object_layer_filter);

    BodyInterface& body_interface = physics_system.GetBodyInterface();

    // Zemin: 100x1x100 statik kutu, üst yüzeyi y=0
    BoxShapeSettings floor_shape_settings(Vec3(50.0f, 0.5f, 50.0f));
    ShapeRefC floor_shape = floor_shape_settings.Create().Get();
    BodyCreationSettings floor_settings(floor_shape, RVec3(0.0_r, -0.5_r, 0.0_r),
                                        Quat::sIdentity(), EMotionType::Static, Layers::NON_MOVING);
    body_interface.CreateAndAddBody(floor_settings, EActivation::DontActivate);

    // Küp: 1m kenar, 10m yükseklikten düşer — adapter üzerinden konumlandır
    alazforge::Vec3 start_pos{0.0f, 10.0f, 0.0f};
    BodyCreationSettings cube_settings(new BoxShape(Vec3(0.5f, 0.5f, 0.5f)),
                                       alazforge::ToJoltR(start_pos), Quat::sIdentity(),
                                       EMotionType::Dynamic, Layers::MOVING);
    BodyID cube_id = body_interface.CreateAndAddBody(cube_settings, EActivation::Activate);

    physics_system.OptimizeBroadPhase();

    const float cDeltaTime = 1.0f / 60.0f;
    int step = 0;
    while (body_interface.IsActive(cube_id) && step < 600) {
        physics_system.Update(cDeltaTime, 1, &temp_allocator, &job_system);
        ++step;

        if (step % 30 == 0) {
            alazforge::Vec3 pos = alazforge::FromJolt(body_interface.GetCenterOfMassPosition(cube_id));
            printf("adim %3d: kup y = %.3f\n", step, pos.y);
        }
    }

    alazforge::Vec3 final_pos = alazforge::FromJolt(body_interface.GetCenterOfMassPosition(cube_id));
    bool asleep = !body_interface.IsActive(cube_id);
    // 1m küpün merkezi zemine oturunca y ~= 0.5 olmalı
    bool resting = std::fabs(final_pos.y - 0.5f) < 0.05f;

    printf("sonuc: %d adimda durdu, son y = %.3f, uykuda = %s\n",
           step, final_pos.y, asleep ? "evet" : "hayir");

    body_interface.RemoveBody(cube_id);
    body_interface.DestroyBody(cube_id);

    UnregisterTypes();
    delete Factory::sInstance;
    Factory::sInstance = nullptr;

    if (asleep && resting) {
        printf("TEST BASARILI: kup yercekimiyle dusup zemine oturdu.\n");
        return 0;
    }
    printf("TEST BASARISIZ!\n");
    return 1;
}
