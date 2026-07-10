#include "vehicle/VehicleSystem.h"

#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/OffsetCenterOfMassShape.h>
#include <Jolt/Physics/Vehicle/TrackedVehicleController.h>
#include <Jolt/Physics/Vehicle/VehicleAntiRollBar.h>
#include <Jolt/Physics/Vehicle/VehicleCollisionTester.h>
#include <Jolt/Physics/Vehicle/WheeledVehicleController.h>

#include <algorithm>
#include <cmath>

namespace alazforge {

namespace {

// Tekerlekli tekerleklere özgü alanlar (direksiyon/fren)
void ScaleFrictionCurve(JPH::LinearCurve& curve, float multiplier) {
    for (JPH::LinearCurve::Point& p : curve.mPoints)
        p.mY *= multiplier;
}

void ConfigureWheelExtra(JPH::WheelSettingsWV* w, const AxleConfig& a) {
    w->mMaxSteerAngle = JPH::DegreesToRadians(a.maxSteerAngleDeg);
    w->mMaxBrakeTorque = a.maxBrakeTorque;
    w->mMaxHandBrakeTorque = a.maxHandBrakeTorque;
    // Jolt'un varsayilan slip egrilerini olcekle: <1 kaygan, >1 yaris lastigi
    ScaleFrictionCurve(w->mLongitudinalFriction, a.tireGripLongitudinal);
    ScaleFrictionCurve(w->mLateralFriction, a.tireGripLateral);
}
// Paletli tekerleklerin direksiyonu yok; skid-steer dönüşünün mümkün
// olması için yanal sürtünme boyunadan düşük tutulur (paletler dönerken
// yana kayar). Aksi halde uzun gövde yalpalamaya direnir.
void ConfigureWheelExtra(JPH::WheelSettingsTV* w, const AxleConfig& a) {
    w->mLongitudinalFriction = 4.0f;
    w->mLateralFriction = a.trackLateralFriction;
}

} // namespace

VehicleSystem::~VehicleSystem() { Destroy(); }

void VehicleSystem::CreateWheeled(JPH::PhysicsSystem& inPhysics,
                                  const VehicleChassisConfig& chassis,
                                  const std::vector<AxleConfig>& axles, const EngineConfig& engine,
                                  const TransmissionConfig& trans, JPH::ObjectLayer testerLayer) {
    Destroy();
    physics = &inPhysics;
    kind = VehicleKind::Wheeled;
    bodyId = CreateChassisBody(chassis, testerLayer);

    JPH::VehicleConstraintSettings vc;
    vc.mMaxPitchRollAngle = JPH::DegreesToRadians(chassis.maxPitchRollAngleDeg);
    BuildWheels<JPH::WheelSettingsWV>(vc, axles);

    // Sürülen aksları diferansiyellere böl
    std::vector<int> drivenAxles;
    for (int i = 0; i < static_cast<int>(axles.size()); ++i)
        if (axles[i].driven) drivenAxles.push_back(i);
    const float torqueShare =
        drivenAxles.empty() ? 0.0f : 1.0f / static_cast<float>(drivenAxles.size());

    auto* controller = new JPH::WheeledVehicleControllerSettings();
    controller->mEngine.mMaxTorque = engine.maxTorque;
    controller->mEngine.mMinRPM = engine.minRPM;
    controller->mEngine.mMaxRPM = engine.maxRPM;
    controller->mTransmission.mGearRatios.assign(trans.gearRatios.begin(), trans.gearRatios.end());
    controller->mTransmission.mReverseGearRatios.assign(trans.reverseGearRatios.begin(),
                                                        trans.reverseGearRatios.end());

    controller->mDifferentials.resize(drivenAxles.size());
    for (size_t d = 0; d < drivenAxles.size(); ++d) {
        const int axle = drivenAxles[d];
        controller->mDifferentials[d].mLeftWheel = axle * 2;
        controller->mDifferentials[d].mRightWheel = axle * 2 + 1;
        controller->mDifferentials[d].mEngineTorqueRatio = torqueShare;
    }
    vc.mController = controller;

    FinalizeConstraint(vc, testerLayer);
}

void VehicleSystem::CreateTracked(JPH::PhysicsSystem& inPhysics,
                                  const VehicleChassisConfig& chassis,
                                  const std::vector<AxleConfig>& axles, const EngineConfig& engine,
                                  const TransmissionConfig& trans, JPH::ObjectLayer testerLayer) {
    Destroy();
    physics = &inPhysics;
    kind = VehicleKind::Tracked;
    bodyId = CreateChassisBody(chassis, testerLayer);

    JPH::VehicleConstraintSettings vc;
    vc.mMaxPitchRollAngle = JPH::DegreesToRadians(chassis.maxPitchRollAngleDeg);
    BuildWheels<JPH::WheelSettingsTV>(vc, axles);

    auto* controller = new JPH::TrackedVehicleControllerSettings();
    controller->mEngine.mMaxTorque = engine.maxTorque;
    controller->mEngine.mMinRPM = engine.minRPM;
    controller->mEngine.mMaxRPM = engine.maxRPM;
    controller->mTransmission.mGearRatios.assign(trans.gearRatios.begin(), trans.gearRatios.end());
    controller->mTransmission.mReverseGearRatios.assign(trans.reverseGearRatios.begin(),
                                                        trans.reverseGearRatios.end());

    // Sol tekerlekler çift indeks (2i), sağ tekerlekler tek indeks (2i+1)
    std::vector<JPH::uint> leftWheels, rightWheels;
    for (int i = 0; i < static_cast<int>(axles.size()); ++i) {
        leftWheels.push_back(static_cast<JPH::uint>(i * 2));
        rightWheels.push_back(static_cast<JPH::uint>(i * 2 + 1));
    }
    JPH::VehicleTrackSettings& left = controller->mTracks[(int)JPH::ETrackSide::Left];
    JPH::VehicleTrackSettings& right = controller->mTracks[(int)JPH::ETrackSide::Right];
    left.mWheels.assign(leftWheels.begin(), leftWheels.end());
    right.mWheels.assign(rightWheels.begin(), rightWheels.end());
    left.mDrivenWheel = leftWheels.front();
    right.mDrivenWheel = rightWheels.front();
    vc.mController = controller;

    FinalizeConstraint(vc, testerLayer);
}

void VehicleSystem::SetDriverInput(float forward, float right, float brake, float handBrake) {
    if (!constraint) return;
    Wake();

    if (kind == VehicleKind::Wheeled) {
        auto* c = static_cast<JPH::WheeledVehicleController*>(constraint->GetController());
        c->SetForwardInput(forward);
        c->SetRightInput(right);
        c->SetBrakeInput(brake);
        c->SetHandBrakeInput(handBrake);
    } else if (kind == VehicleKind::Tracked) {
        auto* c = static_cast<JPH::TrackedVehicleController*>(constraint->GetController());
        // Skid-steer: dönüş için bir paletin oranı azaltılır. Oranların
        // ÇARPIMI pozitif kalmalı; aksi halde Jolt transmisyonu boşa alıp
        // motoru devre dışı bırakır (TrackedVehicleController.cpp). Bu yüzden
        // paletleri ters yönde döndürmek yerine yavaşlatıyoruz.
        float leftRatio = 1.0f, rightRatio = 1.0f;
        if (right > 0.0f)
            rightRatio = std::clamp(1.0f - right, 0.1f, 1.0f); // sağa dön
        else if (right < 0.0f)
            leftRatio = std::clamp(1.0f + right, 0.1f, 1.0f); // sola dön
        c->SetDriverInput(forward, leftRatio, rightRatio, brake);
    }
}

Vec3 VehicleSystem::GetPosition() const {
    return FromJolt(physics->GetBodyInterface().GetCenterOfMassPosition(bodyId));
}

Vec3 VehicleSystem::GetLinearVelocity() const {
    return FromJolt(physics->GetBodyInterface().GetLinearVelocity(bodyId));
}

float VehicleSystem::GetForwardSpeed() const {
    JPH::Vec3 vel = physics->GetBodyInterface().GetLinearVelocity(bodyId);
    JPH::Quat rot = physics->GetBodyInterface().GetRotation(bodyId);
    JPH::Vec3 fwd = rot * JPH::Vec3(0, 0, 1);
    return vel.Dot(fwd);
}

float VehicleSystem::GetHeading() const {
    JPH::Quat rot = physics->GetBodyInterface().GetRotation(bodyId);
    JPH::Vec3 fwd = rot * JPH::Vec3(0, 0, 1);
    return std::atan2(fwd.GetX(), fwd.GetZ());
}

void VehicleSystem::Wake() { physics->GetBodyInterface().ActivateBody(bodyId); }

JPH::BodyID VehicleSystem::CreateChassisBody(const VehicleChassisConfig& chassis,
                                             JPH::ObjectLayer layer) {
    JPH::BoxShapeSettings boxSettings(ToJolt(chassis.halfExtents));
    boxSettings.SetEmbedded();
    JPH::ShapeRefC boxShape = boxSettings.Create().Get();

    // Ağırlık merkezini aşağı al -> devrilmeye karşı stabilite
    const float comDrop = chassis.halfExtents.y * chassis.comLowerRatio;
    JPH::OffsetCenterOfMassShapeSettings comSettings(JPH::Vec3(0, -comDrop, 0), boxShape);
    comSettings.SetEmbedded();
    JPH::ShapeRefC shape = comSettings.Create().Get();

    JPH::BodyCreationSettings body(shape, ToJoltR(chassis.position), JPH::Quat::sIdentity(),
                                   JPH::EMotionType::Dynamic, layer);
    body.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
    body.mMassPropertiesOverride.mMass = chassis.mass;

    return physics->GetBodyInterface().CreateAndAddBody(body, JPH::EActivation::Activate);
}

// Aks listesinden sol/sağ tekerlek ayarlarını üretir (WV veya TV)
template <typename WheelSettingsT>
void VehicleSystem::BuildWheels(JPH::VehicleConstraintSettings& vc,
                                const std::vector<AxleConfig>& axles) {
    for (int i = 0; i < static_cast<int>(axles.size()); ++i) {
        const AxleConfig& a = axles[i];
        for (int side = 0; side < 2; ++side) {
            const float sign = (side == 0) ? -1.0f : 1.0f; // 0=sol, 1=sağ
            auto* w = new WheelSettingsT();
            w->mPosition = JPH::Vec3(sign * a.halfTrackWidth, a.attachHeight, a.positionZ);
            w->mSuspensionDirection = JPH::Vec3(0, -1, 0);
            w->mSuspensionMinLength = a.suspensionMinLength;
            w->mSuspensionMaxLength = a.suspensionMaxLength;
            w->mSuspensionSpring = JPH::SpringSettings(JPH::ESpringMode::FrequencyAndDamping,
                                                       a.suspensionFrequency, a.suspensionDamping);
            w->mRadius = a.wheelRadius;
            w->mWidth = a.wheelWidth;
            ConfigureWheelExtra(w, a);
            vc.mWheels.push_back(w);
        }

        if (a.antiRollStiffness > 0.0f) {
            JPH::VehicleAntiRollBar bar;
            bar.mLeftWheel = i * 2;
            bar.mRightWheel = i * 2 + 1;
            bar.mStiffness = a.antiRollStiffness;
            vc.mAntiRollBars.push_back(bar);
        }
    }
}

// Constraint'i kurar, fizik sistemine + step listener'a kaydeder,
// tekerlek çarpışma test edicisini bağlar.
void VehicleSystem::FinalizeConstraint(JPH::VehicleConstraintSettings& vc,
                                       JPH::ObjectLayer testerLayer) {
    JPH::Body* bodyPtr = nullptr;
    {
        JPH::BodyLockWrite lock(physics->GetBodyLockInterface(), bodyId);
        if (lock.Succeeded()) bodyPtr = &lock.GetBody();
        // Kilit kapsamı dışında constraint kuramayız; işaretçiyi tutmak
        // güvenli çünkü body fizik sisteminde yaşamaya devam ediyor.
    }
    constraint = new JPH::VehicleConstraint(*bodyPtr, vc);

    // Silindir cast: ağır lastiklerin zeminle temasını gerçekçi test eder
    tester = new JPH::VehicleCollisionTesterCastCylinder(testerLayer, 0.05f);
    constraint->SetVehicleCollisionTester(tester);

    physics->AddConstraint(constraint);
    physics->AddStepListener(constraint);
}

void VehicleSystem::Destroy() {
    if (!physics) return;
    if (constraint) {
        physics->RemoveStepListener(constraint);
        physics->RemoveConstraint(constraint);
        constraint = nullptr;
    }
    tester = nullptr;
    if (!bodyId.IsInvalid()) {
        physics->GetBodyInterface().RemoveBody(bodyId);
        physics->GetBodyInterface().DestroyBody(bodyId);
        bodyId = JPH::BodyID();
    }
    kind = VehicleKind::None;
}

} // namespace alazforge
