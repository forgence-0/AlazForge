#include "audio/CollisionEventSystem.h"

namespace alazforge {

CollisionEventSystem::CollisionEventSystem(float minImpactSpeed_)
    : minImpactSpeed(minImpactSpeed_) {}

void CollisionEventSystem::Attach(JPH::PhysicsSystem& physics) { physics.SetContactListener(this); }

void CollisionEventSystem::OnContactAdded(const JPH::Body& body1, const JPH::Body& body2,
                                          const JPH::ContactManifold& manifold,
                                          JPH::ContactSettings&) {
    // Normal dogrultusundaki bagil hiz: yuzeye paralel surtunme temaslarini
    // (arac lastigi yolda) elemek icin esik altindakiler yok sayilir.
    const JPH::Vec3 relVel = body1.GetLinearVelocity() - body2.GetLinearVelocity();
    const float impactSpeed = std::abs(relVel.Dot(manifold.mWorldSpaceNormal));
    if (impactSpeed < minImpactSpeed) return;

    CollisionEvent ev;
    ev.bodyA = body1.GetID();
    ev.bodyB = body2.GetID();
    ev.worldPosition = FromJolt(JPH::Vec3(manifold.mBaseOffset));
    ev.worldNormal = FromJolt(manifold.mWorldSpaceNormal);
    ev.impactSpeed = impactSpeed;
    ev.materialA = static_cast<MaterialId>(body1.GetUserData());
    ev.materialB = static_cast<MaterialId>(body2.GetUserData());

    std::lock_guard<std::mutex> lock(mutex);
    pending.push_back(ev);
}

void CollisionEventSystem::DrainEvents(std::vector<CollisionEvent>& out) {
    std::lock_guard<std::mutex> lock(mutex);
    out = std::move(pending);
    pending.clear();
}

} // namespace alazforge
