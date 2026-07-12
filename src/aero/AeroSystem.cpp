#include "aero/AeroSystem.h"

#include <Jolt/Physics/Body/BodyInterface.h>

namespace alazforge {

void AeroSystem::ApplyParachute(JPH::PhysicsSystem& physics, JPH::BodyID body,
                                const ParachuteConfig& config, const Vec3& wind) {
    JPH::BodyInterface& bi = physics.GetBodyInterface();
    const JPH::Vec3 vel = bi.GetLinearVelocity(body);
    const JPH::Vec3 rel = vel - ToJolt(wind);
    const float speed = rel.Length();
    if (speed < 1.0e-4f) return;

    // Kuadratik surukleme, hiza ters yonde
    const JPH::Vec3 force = rel * (-config.dragCoefficient * speed);
    // Baglanti noktasi govde merkezinin ustunde: cisim kendini diklestirir
    const JPH::RVec3 attachPoint = bi.GetCenterOfMassPosition(body) +
                                   JPH::RVec3(0.0, static_cast<double>(config.attachHeight), 0.0);
    bi.AddForce(body, force, attachPoint);
    bi.ActivateBody(body);
}

void AeroSystem::ApplyGlider(JPH::PhysicsSystem& physics, JPH::BodyID body,
                             const GliderConfig& config, const Vec3& forward, const Vec3& wind) {
    JPH::BodyInterface& bi = physics.GetBodyInterface();
    const JPH::Vec3 vel = bi.GetLinearVelocity(body);
    const JPH::Vec3 rel = vel - ToJolt(wind);
    const float speed = rel.Length();
    if (speed < 1.0e-3f) return;

    const JPH::Vec3 velDir = rel / speed;
    JPH::Vec3 fwd = ToJolt(forward);
    if (fwd.LengthSq() < 1.0e-8f) return;
    fwd = fwd.Normalized();

    // Kanat yan ekseni (dunya-yukari ile ileri yonun dik carpimi); tasima,
    // hava-bagil hiza VE yan eksene dik yonde (kanat "yukarisi") olusur.
    JPH::Vec3 side = fwd.Cross(JPH::Vec3::sAxisY());
    if (side.LengthSq() < 1.0e-6f) side = JPH::Vec3::sAxisX(); // dik dalis durumu
    side = side.Normalized();
    JPH::Vec3 liftDir = side.Cross(velDir);
    if (liftDir.GetY() < 0.0f) liftDir = -liftDir; // tasima daima yukari bileseni tasir
    liftDir = liftDir.Normalized();

    // Dinamik basincla oranli buyuklukler (yogunluk katsayilarin icinde)
    const float q = speed * speed;
    const JPH::Vec3 lift = liftDir * (config.liftCoefficient * q);
    const JPH::Vec3 drag = velDir * (-config.dragCoefficient * q);
    bi.AddForce(body, lift + drag);
    bi.ActivateBody(body);
}

} // namespace alazforge
