#pragma once
// Araç fiziği modülü (Faz 5). Jolt'un VehicleConstraint sistemini temel alır
// ve hem tekerlekli (WheeledVehicleController) hem paletli
// (TrackedVehicleController) ağır vasıtalar için ortak bir sarmalayıcı sunar.
//
// Ağır vasıta odaklı: aks sayısı serbest (çok akslı kamyon/TIR), aks başına
// süspansiyon/fren/direksiyon/traksiyon parametreleri. Motor torku sürüşe açık
// akslara diferansiyeller üzerinden eşit paylaştırılır.
//
// Kullanım:
//   VehicleSystem v;
//   v.CreateWheeled(physics, chassis, axles, engine, trans, movingLayer);
//   ... her frame: v.SetDriverInput(gas, steer, brake, handbrake);
//   ... physics.Update(...) constraint'i step listener olarak sü
//
// SetDriverInput'ta 'right' parametresi tekerlekliyde direksiyon açısına,
// paletlide sol/sağ palet hız oranına (diferansiyel dönüş) eşlenir.

#include "core/JoltAdapter.h"

#include <Jolt/Jolt.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/OffsetCenterOfMassShape.h>
#include <Jolt/Physics/Vehicle/VehicleConstraint.h>
#include <Jolt/Physics/Vehicle/WheeledVehicleController.h>
#include <Jolt/Physics/Vehicle/TrackedVehicleController.h>
#include <Jolt/Physics/Vehicle/VehicleCollisionTester.h>
#include <Jolt/Physics/Vehicle/VehicleAntiRollBar.h>
#include <Jolt/Physics/Body/BodyLock.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace alazforge {

// Tek bir aks (sol+sağ tekerlek çifti) tanımı
struct AxleConfig {
    float positionZ = 0.0f;          // gövde merkezine göre boyuna konum (m, +ileri)
    float attachHeight = 0.0f;       // süspansiyon bağlantı noktası y (gövde uzayı)
    float halfTrackWidth = 1.0f;     // merkez ile tekerlek arası (m)
    float wheelRadius = 0.5f;        // m
    float wheelWidth = 0.35f;        // m
    float suspensionMinLength = 0.3f;
    float suspensionMaxLength = 0.6f;
    float suspensionFrequency = 1.5f; // Hz
    float suspensionDamping = 0.5f;
    float maxSteerAngleDeg = 0.0f;    // 0 = direksiyonsuz aks
    float maxBrakeTorque = 15000.0f;  // Nm
    float maxHandBrakeTorque = 0.0f;  // genelde arka aks
    float antiRollStiffness = 5000.0f;
    float trackLateralFriction = 1.0f; // yalnızca paletli: düşük = kolay dönüş
    bool driven = true;               // motor torku bu aksa ulaşır mı
};

struct EngineConfig {
    float maxTorque = 2500.0f; // Nm (ağır dizel)
    float minRPM = 600.0f;
    float maxRPM = 2400.0f;
};

struct TransmissionConfig {
    // Ağır vasıta için düşük ve çok kademeli aktarma
    std::vector<float> gearRatios = {11.0f, 8.0f, 6.0f, 4.5f, 3.2f, 2.2f, 1.5f, 1.0f};
    std::vector<float> reverseGearRatios = {-9.0f};
};

struct VehicleChassisConfig {
    Vec3 halfExtents{1.25f, 0.7f, 5.0f}; // ~2.5m geniş, 10m uzun gövde
    float mass = 12000.0f;               // kg (yüklü ağır vasıta)
    Vec3 position{0, 3, 0};
    float maxPitchRollAngleDeg = 60.0f;  // devrilme koruması
    float comLowerRatio = 0.7f;          // ağırlık merkezini gövde yarı-yüksekliğinin bu oranı kadar aşağı al
};

enum class VehicleKind : uint8_t { None, Wheeled, Tracked };

class VehicleSystem {
public:
    VehicleSystem() = default;

    // Kopyalamayı yasakla (Jolt kaynaklarını sahipleniyoruz)
    VehicleSystem(const VehicleSystem&) = delete;
    VehicleSystem& operator=(const VehicleSystem&) = delete;

    ~VehicleSystem() { Destroy(); }

    // ── Tekerlekli çok akslı vasıta ─────────────────────────────────
    void CreateWheeled(JPH::PhysicsSystem& inPhysics, const VehicleChassisConfig& chassis,
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
        controller->mTransmission.mReverseGearRatios.assign(
            trans.reverseGearRatios.begin(), trans.reverseGearRatios.end());

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

    // ── Paletli (tırtıl) vasıta ─────────────────────────────────────
    void CreateTracked(JPH::PhysicsSystem& inPhysics, const VehicleChassisConfig& chassis,
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
        controller->mTransmission.mReverseGearRatios.assign(
            trans.reverseGearRatios.begin(), trans.reverseGearRatios.end());

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

    // ── Sürücü girdisi (birleşik) ───────────────────────────────────
    // forward  : -1..1  (gaz / geri)
    // right    : -1..1  (tekerlekli: direksiyon; paletli: diferansiyel dönüş)
    // brake    :  0..1
    // handBrake:  0..1  (paletlide yok sayılır)
    void SetDriverInput(float forward, float right, float brake, float handBrake = 0.0f) {
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
                rightRatio = std::clamp(1.0f - right, 0.1f, 1.0f);   // sağa dön
            else if (right < 0.0f)
                leftRatio = std::clamp(1.0f + right, 0.1f, 1.0f);    // sola dön
            c->SetDriverInput(forward, leftRatio, rightRatio, brake);
        }
    }

    // ── Sorgular ────────────────────────────────────────────────────
    JPH::BodyID GetBodyID() const { return bodyId; }
    VehicleKind Kind() const { return kind; }

    Vec3 GetPosition() const {
        return FromJolt(physics->GetBodyInterface().GetCenterOfMassPosition(bodyId));
    }

    Vec3 GetLinearVelocity() const {
        return FromJolt(physics->GetBodyInterface().GetLinearVelocity(bodyId));
    }

    float GetForwardSpeed() const {
        JPH::Vec3 vel = physics->GetBodyInterface().GetLinearVelocity(bodyId);
        JPH::Quat rot = physics->GetBodyInterface().GetRotation(bodyId);
        JPH::Vec3 fwd = rot * JPH::Vec3(0, 0, 1);
        return vel.Dot(fwd);
    }

    // Gövdenin dünya uzayındaki ileri yön açısı (radyan, y ekseni etrafında)
    float GetHeading() const {
        JPH::Quat rot = physics->GetBodyInterface().GetRotation(bodyId);
        JPH::Vec3 fwd = rot * JPH::Vec3(0, 0, 1);
        return std::atan2(fwd.GetX(), fwd.GetZ());
    }

    JPH::VehicleConstraint* Constraint() { return constraint; }

private:
    JPH::PhysicsSystem* physics = nullptr;
    JPH::BodyID bodyId;
    JPH::Ref<JPH::VehicleConstraint> constraint;
    JPH::Ref<JPH::VehicleCollisionTester> tester;
    VehicleKind kind = VehicleKind::None;

    void Wake() {
        physics->GetBodyInterface().ActivateBody(bodyId);
    }

    JPH::BodyID CreateChassisBody(const VehicleChassisConfig& chassis, JPH::ObjectLayer layer) {
        JPH::BoxShapeSettings boxSettings(ToJolt(chassis.halfExtents));
        boxSettings.SetEmbedded();
        JPH::ShapeRefC boxShape = boxSettings.Create().Get();

        // Ağırlık merkezini aşağı al -> devrilmeye karşı stabilite
        const float comDrop = chassis.halfExtents.y * chassis.comLowerRatio;
        JPH::OffsetCenterOfMassShapeSettings comSettings(JPH::Vec3(0, -comDrop, 0), boxShape);
        comSettings.SetEmbedded();
        JPH::ShapeRefC shape = comSettings.Create().Get();

        JPH::BodyCreationSettings body(shape, ToJoltR(chassis.position),
                                       JPH::Quat::sIdentity(), JPH::EMotionType::Dynamic, layer);
        body.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
        body.mMassPropertiesOverride.mMass = chassis.mass;

        return physics->GetBodyInterface().CreateAndAddBody(body, JPH::EActivation::Activate);
    }

    // Aks listesinden sol/sağ tekerlek ayarlarını üretir (WV veya TV)
    template <typename WheelSettingsT>
    void BuildWheels(JPH::VehicleConstraintSettings& vc, const std::vector<AxleConfig>& axles) {
        for (int i = 0; i < static_cast<int>(axles.size()); ++i) {
            const AxleConfig& a = axles[i];
            for (int side = 0; side < 2; ++side) {
                const float sign = (side == 0) ? -1.0f : 1.0f; // 0=sol, 1=sağ
                auto* w = new WheelSettingsT();
                w->mPosition = JPH::Vec3(sign * a.halfTrackWidth, a.attachHeight, a.positionZ);
                w->mSuspensionDirection = JPH::Vec3(0, -1, 0);
                w->mSuspensionMinLength = a.suspensionMinLength;
                w->mSuspensionMaxLength = a.suspensionMaxLength;
                w->mSuspensionSpring = JPH::SpringSettings(
                    JPH::ESpringMode::FrequencyAndDamping, a.suspensionFrequency, a.suspensionDamping);
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

    // Tekerlekli tekerleklere özgü alanlar (direksiyon/fren)
    static void ConfigureWheelExtra(JPH::WheelSettingsWV* w, const AxleConfig& a) {
        w->mMaxSteerAngle = JPH::DegreesToRadians(a.maxSteerAngleDeg);
        w->mMaxBrakeTorque = a.maxBrakeTorque;
        w->mMaxHandBrakeTorque = a.maxHandBrakeTorque;
    }
    // Paletli tekerleklerin direksiyonu yok; skid-steer dönüşünün mümkün
    // olması için yanal sürtünme boyunadan düşük tutulur (paletler dönerken
    // yana kayar). Aksi halde uzun gövde yalpalamaya direnir.
    static void ConfigureWheelExtra(JPH::WheelSettingsTV* w, const AxleConfig& a) {
        w->mLongitudinalFriction = 4.0f;
        w->mLateralFriction = a.trackLateralFriction;
    }

    // Constraint'i kurar, fizik sistemine + step listener'a kaydeder,
    // tekerlek çarpışma test edicisini bağlar.
    void FinalizeConstraint(JPH::VehicleConstraintSettings& vc, JPH::ObjectLayer testerLayer) {
        JPH::Body* bodyPtr = nullptr;
        {
            JPH::BodyLockWrite lock(physics->GetBodyLockInterface(), bodyId);
            if (lock.Succeeded())
                bodyPtr = &lock.GetBody();
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

    void Destroy() {
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
};

} // namespace alazforge
