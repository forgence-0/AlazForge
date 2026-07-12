#include "scene/SceneFile.h"

#include "core/JoltAdapter.h"
#include "scene/MiniJson.h"

#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>

#include <fstream>
#include <sstream>

namespace alazforge {

namespace {

const char* ShapeTypeName(SceneShapeType t) {
    switch (t) {
        case SceneShapeType::Box:
            return "box";
        case SceneShapeType::Sphere:
            return "sphere";
        case SceneShapeType::Capsule:
            return "capsule";
    }
    return "box";
}

SceneShapeType ShapeTypeFromName(const std::string& s) {
    if (s == "sphere") return SceneShapeType::Sphere;
    if (s == "capsule") return SceneShapeType::Capsule;
    return SceneShapeType::Box;
}

JsonValue Vec3ToJson(const Vec3& v) {
    JsonArray a = {JsonValue(static_cast<double>(v.x)), JsonValue(static_cast<double>(v.y)),
                   JsonValue(static_cast<double>(v.z))};
    return JsonValue(a);
}

Vec3 Vec3FromJson(const JsonValue& v, Vec3 def) {
    const JsonArray& a = v.AsArray();
    if (a.size() != 3) return def;
    return Vec3{static_cast<f32>(a[0].AsNumber()), static_cast<f32>(a[1].AsNumber()),
                static_cast<f32>(a[2].AsNumber())};
}

JsonValue QuatToJson(const Quat& q) {
    JsonArray a = {JsonValue(static_cast<double>(q.x)), JsonValue(static_cast<double>(q.y)),
                   JsonValue(static_cast<double>(q.z)), JsonValue(static_cast<double>(q.w))};
    return JsonValue(a);
}

Quat QuatFromJson(const JsonValue& v, Quat def) {
    const JsonArray& a = v.AsArray();
    if (a.size() != 4) return def;
    return Quat{static_cast<f32>(a[0].AsNumber()), static_cast<f32>(a[1].AsNumber()),
                static_cast<f32>(a[2].AsNumber()), static_cast<f32>(a[3].AsNumber())};
}

} // namespace

bool SaveSceneFile(const std::string& path, const SceneDefinition& scene) {
    JsonValue root;
    root.Set("gravity", Vec3ToJson(scene.gravity));

    JsonValue bodies = JsonValue::MakeArray();
    for (const SceneBodyDef& b : scene.bodies) {
        JsonValue jb;
        jb.Set("shape", JsonValue(ShapeTypeName(b.shape)));
        jb.Set("halfExtents", Vec3ToJson(b.halfExtents));
        jb.Set("radius", JsonValue(static_cast<double>(b.radius)));
        jb.Set("halfHeight", JsonValue(static_cast<double>(b.halfHeight)));
        jb.Set("position", Vec3ToJson(b.position));
        jb.Set("rotation", QuatToJson(b.rotation));
        jb.Set("isStatic", JsonValue(b.isStatic));
        jb.Set("layer", JsonValue(static_cast<double>(b.layer)));
        jb.Set("material", JsonValue(static_cast<double>(b.material)));
        jb.Set("massOverride", JsonValue(static_cast<double>(b.massOverride)));
        bodies.PushBack(jb);
    }
    root.Set("bodies", bodies);

    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    if (!out.is_open()) return false;
    out << root.Dump(0) << "\n";
    return out.good();
}

bool LoadSceneFile(const std::string& path, SceneDefinition& outScene) {
    std::ifstream in(path, std::ios::binary);
    if (!in.is_open()) return false;
    std::ostringstream buf;
    buf << in.rdbuf();

    JsonValue root;
    if (!ParseJson(buf.str(), root)) return false;
    if (root.Type() != JsonType::Object) return false;

    SceneDefinition scene;
    scene.gravity = Vec3FromJson(root["gravity"], scene.gravity);

    for (const JsonValue& jb : root["bodies"].AsArray()) {
        SceneBodyDef b;
        b.shape = ShapeTypeFromName(jb["shape"].AsString());
        b.halfExtents = Vec3FromJson(jb["halfExtents"], b.halfExtents);
        b.radius = static_cast<f32>(jb["radius"].AsNumber(b.radius));
        b.halfHeight = static_cast<f32>(jb["halfHeight"].AsNumber(b.halfHeight));
        b.position = Vec3FromJson(jb["position"], b.position);
        b.rotation = QuatFromJson(jb["rotation"], b.rotation);
        b.isStatic = jb["isStatic"].AsBool(b.isStatic);
        b.layer = static_cast<JPH::ObjectLayer>(jb["layer"].AsNumber(b.layer));
        b.material = static_cast<MaterialId>(jb["material"].AsNumber(b.material));
        b.massOverride = static_cast<f32>(jb["massOverride"].AsNumber(b.massOverride));
        scene.bodies.push_back(b);
    }

    outScene = std::move(scene);
    return true;
}

std::vector<JPH::BodyID> InstantiateScene(JPH::PhysicsSystem& physics,
                                          const SceneDefinition& scene) {
    physics.SetGravity(ToJolt(scene.gravity));

    std::vector<JPH::BodyID> ids;
    ids.reserve(scene.bodies.size());
    JPH::BodyInterface& bi = physics.GetBodyInterface();

    for (const SceneBodyDef& b : scene.bodies) {
        JPH::RefConst<JPH::Shape> shape;
        switch (b.shape) {
            case SceneShapeType::Box:
                shape = new JPH::BoxShape(ToJolt(b.halfExtents));
                break;
            case SceneShapeType::Sphere:
                shape = new JPH::SphereShape(b.radius);
                break;
            case SceneShapeType::Capsule:
                shape = new JPH::CapsuleShape(b.halfHeight, b.radius);
                break;
        }

        const JPH::EMotionType motion =
            b.isStatic ? JPH::EMotionType::Static : JPH::EMotionType::Dynamic;
        JPH::BodyCreationSettings settings(shape, ToJoltR(b.position), ToJolt(b.rotation), motion,
                                           b.layer);
        settings.mUserData = b.material;
        if (!b.isStatic && b.massOverride > 0.0f) {
            settings.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
            settings.mMassPropertiesOverride.mMass = b.massOverride;
        }

        const JPH::EActivation activation =
            b.isStatic ? JPH::EActivation::DontActivate : JPH::EActivation::Activate;
        ids.push_back(bi.CreateAndAddBody(settings, activation));
    }

    return ids;
}

} // namespace alazforge
