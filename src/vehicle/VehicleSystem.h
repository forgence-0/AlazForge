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

#include <physics/export.h>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Vehicle/VehicleConstraint.h>

#include <vector>

namespace alazforge {

// Tek bir aks (sol+sağ tekerlek çifti) tanımı
struct AxleConfig {
    float positionZ = 0.0f;      // gövde merkezine göre boyuna konum (m, +ileri)
    float attachHeight = 0.0f;   // süspansiyon bağlantı noktası y (gövde uzayı)
    float halfTrackWidth = 1.0f; // merkez ile tekerlek arası (m)
    float wheelRadius = 0.5f;    // m
    float wheelWidth = 0.35f;    // m
    float suspensionMinLength = 0.3f;
    float suspensionMaxLength = 0.6f;
    float suspensionFrequency = 1.5f; // Hz
    float suspensionDamping = 0.5f;
    float maxSteerAngleDeg = 0.0f;   // 0 = direksiyonsuz aks
    float maxBrakeTorque = 15000.0f; // Nm
    float maxHandBrakeTorque = 0.0f; // genelde arka aks
    float antiRollStiffness = 5000.0f;
    float trackLateralFriction = 1.0f; // yalnızca paletli: düşük = kolay dönüş

    // Lastik tutunması (yalnızca tekerlekli): Jolt'un varsayılan kayma
    // (slip) eğrileri bu çarpanlarla ölçeklenir. 1 = varsayılan tutuş;
    // <1 = kaygan (ıslak/karlı zemin, drift arabası), >1 = yarış lastiği.
    // Yanal (lateral) düşükken araç virajda savrulur; boyuna
    // (longitudinal) düşükken kalkışta patinaj yapar. Zemin gövdesinin
    // friction'ı da çarpıma girer (Jolt CombineFriction) — buz üstünde
    // MaterialDatabase::ApplyToBody("ice") uygulanmış zemin, lastikten
    // bağımsız olarak tutuşu düşürür.
    float tireGripLongitudinal = 1.0f;
    float tireGripLateral = 1.0f;
    bool driven = true; // motor torku bu aksa ulaşır mı
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
    float maxPitchRollAngleDeg = 60.0f; // devrilme koruması
    float comLowerRatio =
        0.7f; // ağırlık merkezini gövde yarı-yüksekliğinin bu oranı kadar aşağı al
};

enum class VehicleKind : uint8_t { None, Wheeled, Tracked };

class PHYSICS_API VehicleSystem {
public:
    VehicleSystem() = default;

    // Kopyalamayı yasakla (Jolt kaynaklarını sahipleniyoruz)
    VehicleSystem(const VehicleSystem&) = delete;
    VehicleSystem& operator=(const VehicleSystem&) = delete;

    ~VehicleSystem();

    // ── Tekerlekli çok akslı vasıta ─────────────────────────────────
    void CreateWheeled(JPH::PhysicsSystem& inPhysics, const VehicleChassisConfig& chassis,
                       const std::vector<AxleConfig>& axles, const EngineConfig& engine,
                       const TransmissionConfig& trans, JPH::ObjectLayer testerLayer);

    // ── Paletli (tırtıl) vasıta ─────────────────────────────────────
    void CreateTracked(JPH::PhysicsSystem& inPhysics, const VehicleChassisConfig& chassis,
                       const std::vector<AxleConfig>& axles, const EngineConfig& engine,
                       const TransmissionConfig& trans, JPH::ObjectLayer testerLayer);

    // ── Sürücü girdisi (birleşik) ───────────────────────────────────
    // forward  : -1..1  (gaz / geri)
    // right    : -1..1  (tekerlekli: direksiyon; paletli: diferansiyel dönüş)
    // brake    :  0..1
    // handBrake:  0..1  (paletlide yok sayılır)
    void SetDriverInput(float forward, float right, float brake, float handBrake = 0.0f);

    // ── Sorgular ────────────────────────────────────────────────────
    JPH::BodyID GetBodyID() const { return bodyId; }
    VehicleKind Kind() const { return kind; }

    Vec3 GetPosition() const;
    Vec3 GetLinearVelocity() const;
    float GetForwardSpeed() const;

    // Gövdenin dünya uzayındaki ileri yön açısı (radyan, y ekseni etrafında)
    float GetHeading() const;

    JPH::VehicleConstraint* Constraint() { return constraint; }

    // ── Hasar modeli ────────────────────────────────────────────────
    // impactSpeed (m/s, çarpışma normali doğrultusunda bağıl hız —
    // CollisionEventSystem::CollisionEvent::impactSpeed ile doğrudan
    // uyumlu) kadar hasar biriktirir. Motor gücü, direksiyon açısı ve
    // fren torku sağlıkla orantılı azalır (0 sağlık = araç sürülemez
    // hale gelir, WreckPersistence'a devredilmeye hazır).
    void ApplyImpactDamage(float impactSpeed);
    float GetHealth() const { return health; } // 1 = sağlam, 0 = hurda
    bool IsDrivable() const { return health > 0.05f; }

private:
    JPH::PhysicsSystem* physics = nullptr;
    JPH::BodyID bodyId;
    JPH::Ref<JPH::VehicleConstraint> constraint;
    JPH::Ref<JPH::VehicleCollisionTester> tester;
    VehicleKind kind = VehicleKind::None;
    float health = 1.0f;
    float baseEngineTorque = 0.0f; // hasarsız motor torku (yuzde hesaplamak icin)

    void Wake();
    JPH::BodyID CreateChassisBody(const VehicleChassisConfig& chassis, JPH::ObjectLayer layer);

    template <typename WheelSettingsT>
    void BuildWheels(JPH::VehicleConstraintSettings& vc, const std::vector<AxleConfig>& axles);

    void FinalizeConstraint(JPH::VehicleConstraintSettings& vc, JPH::ObjectLayer testerLayer);
    void Destroy();
};

} // namespace alazforge
