#include "weapons/TurretMount.h"

#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>

#include <algorithm>
#include <cmath>

namespace alazforge {

namespace {

constexpr float kPi = 3.14159265358979323846f;

// -pi..pi araligina sarilmis en kisa yol farki (yaw tam turluyken 359->1
// derece gibi durumlarda ters yone donmeyi onler).
float ShortestAngleDelta(float from, float to) {
    float delta = std::fmod(to - from, 2.0f * kPi);
    if (delta > kPi) delta -= 2.0f * kPi;
    if (delta < -kPi) delta += 2.0f * kPi;
    return delta;
}

// Orantili kontrol: hata buyukse max hizda, son 0.1 radyanlik bantta
// dogrusal yavaslayarak yaklas (motor hedefe carpip asiri salinmasin diye).
float ClampedRate(float error, float maxRate) {
    constexpr float kBand = 0.1f;
    const float mag = std::min(std::fabs(error) / kBand, 1.0f) * maxRate;
    return error >= 0.0f ? mag : -mag;
}

} // namespace

TurretMount::TurretMount(const TurretMountConfig& inConfig) : config(inConfig) {}

TurretMount::~TurretMount() { Destroy(); }

void TurretMount::AttachToBody(JPH::PhysicsSystem& inPhysics, JPH::BodyID parentBody,
                               JPH::ObjectLayer layer) {
    Destroy();
    physics = &inPhysics;

    JPH::BodyInterface& bi = physics->GetBodyInterface();
    const JPH::RVec3 parentPos = bi.GetPosition(parentBody);
    const JPH::Quat parentRot = bi.GetRotation(parentBody);
    const JPH::RVec3 pivotPos = parentPos + parentRot * ToJolt(config.localOffset);

    // Yaw ve pitch govde kutularinin yari-boylari (Z ekseni): ikisi de
    // pivotta merkezlenirse tamamen ic ice girip collision gurultusu
    // pitch motorunu bastirir (yaw statik govdeye bagli oldugu icin bu
    // gurultuye ragmen hedefe ulasabiliyordu, pitch ulasamiyordu). Pitch
    // govdesini bu iki yari-boyun toplami kadar pivottan one kaydirarak
    // cakismayi gideriyoruz -- pivot ile namlu ucu arasindaki mesafe,
    // her iki eksen de AYNI pivot noktasindan gectigi icin acidan
    // bagimsiz sabit kalir (rijit govde donel hareketinin dogal sonucu).
    constexpr float kYawHalfLengthZ = 0.3f;
    constexpr float kPitchHalfLengthZ = 0.6f;
    constexpr float kPitchForwardOffset = kYawHalfLengthZ + kPitchHalfLengthZ;

    // ── Yaw platformu (dikey eksen etrafinda döner) ─────────────────
    JPH::BoxShapeSettings yawShapeSettings(JPH::Vec3(0.3f, 0.15f, kYawHalfLengthZ));
    yawShapeSettings.SetEmbedded();
    JPH::BodyCreationSettings yawBodySettings(yawShapeSettings.Create().Get(), pivotPos, parentRot,
                                              JPH::EMotionType::Dynamic, layer);
    yawBodySettings.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
    yawBodySettings.mMassPropertiesOverride.mMass = 200.0f;
    yawBody = bi.CreateAndAddBody(yawBodySettings, JPH::EActivation::Activate);

    JPH::HingeConstraintSettings yawSettings;
    yawSettings.mSpace = JPH::EConstraintSpace::WorldSpace;
    yawSettings.mPoint1 = yawSettings.mPoint2 = pivotPos;
    yawSettings.mHingeAxis1 = yawSettings.mHingeAxis2 = parentRot * JPH::Vec3(0, 1, 0);
    yawSettings.mNormalAxis1 = yawSettings.mNormalAxis2 = parentRot * JPH::Vec3(0, 0, 1);
    if (config.yawLimited) {
        yawSettings.mLimitsMin = JPH::DegreesToRadians(config.minYawDeg);
        yawSettings.mLimitsMax = JPH::DegreesToRadians(config.maxYawDeg);
    } else {
        yawSettings.mLimitsMin = -1.0e6f;
        yawSettings.mLimitsMax = 1.0e6f;
    }
    yawSettings.mMotorSettings.mMinTorqueLimit = -config.yawMotorMaxTorqueNm;
    yawSettings.mMotorSettings.mMaxTorqueLimit = config.yawMotorMaxTorqueNm;

    JPH::Body* parentPtr = nullptr;
    {
        JPH::BodyLockWrite lock(physics->GetBodyLockInterface(), parentBody);
        if (lock.Succeeded()) parentPtr = &lock.GetBody();
    }
    JPH::Body* yawPtr = nullptr;
    {
        JPH::BodyLockWrite lock(physics->GetBodyLockInterface(), yawBody);
        if (lock.Succeeded()) yawPtr = &lock.GetBody();
    }
    yawConstraint = static_cast<JPH::HingeConstraint*>(yawSettings.Create(*parentPtr, *yawPtr));
    yawConstraint->SetMotorState(JPH::EMotorState::Velocity);
    physics->AddConstraint(yawConstraint);

    // ── Pitch/namlu gövdesi (yaw platformunun yerel X ekseni etrafında) ──
    JPH::BoxShapeSettings pitchShapeSettings(JPH::Vec3(0.15f, 0.15f, kPitchHalfLengthZ));
    pitchShapeSettings.SetEmbedded();
    const JPH::RVec3 pitchBodyOrigin = pivotPos + parentRot * JPH::Vec3(0, 0, kPitchForwardOffset);
    JPH::BodyCreationSettings pitchBodySettings(pitchShapeSettings.Create().Get(), pitchBodyOrigin,
                                                parentRot, JPH::EMotionType::Dynamic, layer);
    pitchBodySettings.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
    pitchBodySettings.mMassPropertiesOverride.mMass = 50.0f;
    pitchBody = bi.CreateAndAddBody(pitchBodySettings, JPH::EActivation::Activate);

    JPH::HingeConstraintSettings pitchSettings;
    pitchSettings.mSpace = JPH::EConstraintSpace::WorldSpace;
    pitchSettings.mPoint1 = pitchSettings.mPoint2 = pivotPos;
    pitchSettings.mHingeAxis1 = pitchSettings.mHingeAxis2 = parentRot * JPH::Vec3(1, 0, 0);
    pitchSettings.mNormalAxis1 = pitchSettings.mNormalAxis2 = parentRot * JPH::Vec3(0, 0, 1);
    pitchSettings.mLimitsMin = JPH::DegreesToRadians(config.minPitchDeg);
    pitchSettings.mLimitsMax = JPH::DegreesToRadians(config.maxPitchDeg);
    pitchSettings.mMotorSettings.mMinTorqueLimit = -config.pitchMotorMaxTorqueNm;
    pitchSettings.mMotorSettings.mMaxTorqueLimit = config.pitchMotorMaxTorqueNm;

    JPH::Body* yawPtr2 = nullptr;
    {
        JPH::BodyLockWrite lock(physics->GetBodyLockInterface(), yawBody);
        if (lock.Succeeded()) yawPtr2 = &lock.GetBody();
    }
    JPH::Body* pitchPtr = nullptr;
    {
        JPH::BodyLockWrite lock(physics->GetBodyLockInterface(), pitchBody);
        if (lock.Succeeded()) pitchPtr = &lock.GetBody();
    }
    pitchConstraint = static_cast<JPH::HingeConstraint*>(pitchSettings.Create(*yawPtr2, *pitchPtr));
    pitchConstraint->SetMotorState(JPH::EMotorState::Velocity);
    physics->AddConstraint(pitchConstraint);
}

void TurretMount::SetTargetAngles(float targetYawRad, float targetPitchRad) {
    if (!yawConstraint || !pitchConstraint) return;

    const float yawMaxRate = JPH::DegreesToRadians(config.yawRateDegPerSec);
    const float pitchMaxRate = JPH::DegreesToRadians(config.pitchRateDegPerSec);

    const float yawError = ShortestAngleDelta(CurrentYaw(), targetYawRad);
    const float pitchError = targetPitchRad - CurrentPitch();

    yawConstraint->SetTargetAngularVelocity(ClampedRate(yawError, yawMaxRate));
    // NOT: pitch icin JPH::HingeConstraint::GetCurrentAngle()'in isaret yonu,
    // mHingeAxis1 = +X etrafinda SetTargetAngularVelocity'nin fiziksel donus
    // yonunun TERSI cikiyor (CI'da olculdu: pozitif hedef hiz -> negatif aci
    // sonucu). Motor hedefini bu yuzden negatif isaretle veriyoruz; limitler
    // (mLimitsMin/Max) ve GetCurrentAngle() zaten dogru/tutarli oldugu icin
    // (limitin -10'unda kenetlendi) yalnizca motor yonunu ters ceviriyoruz.
    pitchConstraint->SetTargetAngularVelocity(-ClampedRate(pitchError, pitchMaxRate));

    JPH::BodyInterface& bi = physics->GetBodyInterface();
    bi.ActivateBody(yawBody);
    bi.ActivateBody(pitchBody);
}

float TurretMount::CurrentYaw() const {
    return yawConstraint ? yawConstraint->GetCurrentAngle() : 0.0f;
}

float TurretMount::CurrentPitch() const {
    return pitchConstraint ? pitchConstraint->GetCurrentAngle() : 0.0f;
}

Transform TurretMount::GetMuzzleWorldTransform() const {
    if (!physics || pitchBody.IsInvalid()) return Transform{};

    JPH::BodyInterface& bi = physics->GetBodyInterface();
    const JPH::RVec3 pos = bi.GetPosition(pitchBody);
    const JPH::Quat rot = bi.GetRotation(pitchBody);
    const JPH::RVec3 muzzlePos = pos + rot * ToJolt(config.localMuzzleOffset);

    Transform t;
    t.position = FromJolt(JPH::Vec3(muzzlePos));
    t.rotation = FromJolt(rot);
    return t;
}

void TurretMount::Destroy() {
    if (!physics) return;
    JPH::BodyInterface& bi = physics->GetBodyInterface();

    if (pitchConstraint) {
        physics->RemoveConstraint(pitchConstraint);
        pitchConstraint = nullptr;
    }
    if (yawConstraint) {
        physics->RemoveConstraint(yawConstraint);
        yawConstraint = nullptr;
    }
    if (!pitchBody.IsInvalid()) {
        bi.RemoveBody(pitchBody);
        bi.DestroyBody(pitchBody);
        pitchBody = JPH::BodyID();
    }
    if (!yawBody.IsInvalid()) {
        bi.RemoveBody(yawBody);
        bi.DestroyBody(yawBody);
        yawBody = JPH::BodyID();
    }
    physics = nullptr;
}

} // namespace alazforge
