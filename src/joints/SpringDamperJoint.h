#pragma once
// Genel amaçlı yay/damper eklemi: iki gövde arasında bir dinlenme
// mesafesinde tutan, o mesafeden sapıldığında yay kuvveti + sönümleme
// uygulayan bağlantı. Jolt'un DistanceConstraint'i, min=max=dinlenme
// mesafesi ve mLimitsSpringSettings ile kurularak gerçek fizik motoru
// çözücüsünden geçer (kendi yay integrasyonumuzu yazmıyoruz).
//
// Kullanım alanları: araç gövdesi süspansiyonu dışı yay bağlantıları,
// sarkaç/yay tuzakları, esnek montaj noktaları (örn. bir silahı gövdeye
// tamamen sert değil, yaylı bağlamak).

#include "core/JoltAdapter.h"

#include <joints/export.h>

#include <Jolt/Jolt.h>

#include <Jolt/Physics/Constraints/Constraint.h>
#include <Jolt/Physics/PhysicsSystem.h>

namespace alazforge {

struct SpringDamperJointConfig {
    // Dinlenme mesafesi (m). Negatifse Create() çağrısındaki iki dünya
    // çapa noktası arasındaki başlangıç mesafesi kullanılır.
    float restLength = -1.0f;

    // JPH::ESpringMode::FrequencyAndDamping: salınım frekansı (Hz).
    // 0 verilirse eklem tamamen sert (yaysız) davranır.
    float frequencyHz = 4.0f;

    // Sönümleme oranı (0 = sönümsüz salınır, 1 = kritik sönümlü).
    float damping = 0.5f;
};

class JOINTS_API SpringDamperJoint {
public:
    explicit SpringDamperJoint(const SpringDamperJointConfig& inConfig = {});
    ~SpringDamperJoint();

    SpringDamperJoint(const SpringDamperJoint&) = delete;
    SpringDamperJoint& operator=(const SpringDamperJoint&) = delete;

    // İki gövdeyi verilen dünya-uzayı çapa noktalarından yay/damper ile
    // bağlar. Önceki bağlantı varsa önce kaldırılır.
    void Create(JPH::PhysicsSystem& physics, JPH::BodyID bodyA, JPH::BodyID bodyB,
                const Vec3& worldAnchorA, const Vec3& worldAnchorB);

    void Destroy();
    bool IsCreated() const { return constraint != nullptr; }

    // Çapa noktaları arasındaki güncel mesafe (m). Gövdelerin o anki
    // pozisyon/rotasyonuna göre yeniden hesaplanır.
    float CurrentLength() const;

    // Son çözülen alt-adımdan yaklaşık gerginlik kuvveti (N). force =
    // lambda/dt (RopeSystem'in kopma tespitiyle aynı yöntem).
    float CurrentTensionForce(float dt) const;

private:
    SpringDamperJointConfig config;
    JPH::PhysicsSystem* physics = nullptr;
    JPH::Ref<JPH::Constraint> constraint;
    JPH::BodyID bodyA;
    JPH::BodyID bodyB;
    JPH::Vec3 localAnchorA = JPH::Vec3::sZero(); // bodyA yerel uzayında (govde kokeninden)
    JPH::Vec3 localAnchorB = JPH::Vec3::sZero(); // bodyB yerel uzayında
};

} // namespace alazforge
