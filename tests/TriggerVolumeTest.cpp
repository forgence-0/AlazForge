// TriggerVolumeSystem dogrulama testi: bir gövde hacmin disindan icine,
// sonra tekrar disina hareket ettirilir; tam olarak bir Entered ve bir
// Exited olayi uretilmeli (surekli icerideyken/disariyken tekrar olay
// uretilmemeli).

#include "JoltTestWorld.h"
#include "zones/TriggerVolumeSystem.h"

#include <cstdio>
#include <vector>

using namespace alazforge;
using namespace alazforge_test;

static int g_failCount = 0;

#define CHECK(cond, msg)                                      \
    do {                                                      \
        if (cond) {                                           \
            printf("  OK  : %s\n", msg);                      \
        } else {                                              \
            printf("  FAIL: %s (satir %d)\n", msg, __LINE__); \
            ++g_failCount;                                    \
        }                                                     \
    } while (0)

static JPH::BodyID AddKinematicBox(TestWorld& world, JPH::RVec3 pos) {
    JPH::BodyCreationSettings settings(new JPH::BoxShape(JPH::Vec3(0.2f, 0.2f, 0.2f)), pos,
                                       JPH::Quat::sIdentity(), JPH::EMotionType::Kinematic,
                                       Layers::MOVING);
    return world.physics.GetBodyInterface().CreateAndAddBody(settings, JPH::EActivation::Activate);
}

int main() {
    TestWorld world;
    printf("[TriggerVolumeSystem]\n");

    JPH::BodyID body = AddKinematicBox(world, JPH::RVec3(-10, 0, 0));

    TriggerVolumeSystem triggers;
    TriggerVolume vol;
    vol.boundsMin = Vec3{-1, -1, -1};
    vol.boundsMax = Vec3{1, 1, 1};
    vol.userTag = 42;
    const size_t volIndex = triggers.AddVolume(vol);
    triggers.TrackBody(body);

    std::vector<TriggerEvent> events;

    // Disarida -> hala disarida: olay olmamali
    world.physics.GetBodyInterface().SetPosition(body, JPH::RVec3(-8, 0, 0),
                                                 JPH::EActivation::Activate);
    triggers.Update(world.physics, events);
    CHECK(events.empty(), "hacim disindayken olay uretilmiyor");
    CHECK(!triggers.IsBodyInVolume(body, volIndex), "IsBodyInVolume() disardayken false");

    // Icine gir
    world.physics.GetBodyInterface().SetPosition(body, JPH::RVec3(0, 0, 0),
                                                 JPH::EActivation::Activate);
    triggers.Update(world.physics, events);
    CHECK(events.size() == 1, "hacme girince tam 1 olay uretildi");
    if (!events.empty()) {
        CHECK(events[0].type == TriggerEventType::Entered, "olay tipi Entered");
        CHECK(events[0].userTag == 42, "userTag dogru tasindi");
        CHECK(events[0].volumeIndex == volIndex, "volumeIndex dogru");
    }
    CHECK(triggers.IsBodyInVolume(body, volIndex), "IsBodyInVolume() icerdeyken true");

    // Icerde kal: tekrar olay uretilmemeli
    events.clear();
    triggers.Update(world.physics, events);
    CHECK(events.empty(), "icerde kalirken tekrar olay uretilmiyor");

    // Disari cik
    world.physics.GetBodyInterface().SetPosition(body, JPH::RVec3(-8, 0, 0),
                                                 JPH::EActivation::Activate);
    triggers.Update(world.physics, events);
    CHECK(events.size() == 1, "hacimden cikinca tam 1 olay uretildi");
    if (!events.empty()) CHECK(events[0].type == TriggerEventType::Exited, "olay tipi Exited");

    // UntrackBody sonrasi hic olay uretilmemeli
    triggers.UntrackBody(body);
    events.clear();
    world.physics.GetBodyInterface().SetPosition(body, JPH::RVec3(0, 0, 0),
                                                 JPH::EActivation::Activate);
    triggers.Update(world.physics, events);
    CHECK(events.empty(), "UntrackBody sonrasi olay uretilmiyor");

    printf("[TriggerVolumeSystem] %s (%d hata)\n", g_failCount == 0 ? "GECTI" : "BASARISIZ",
           g_failCount);
    return g_failCount == 0 ? 0 : 1;
}
