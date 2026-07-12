#include "material_db/MaterialDatabase.h"

#include <Jolt/Physics/Body/BodyInterface.h>

namespace alazforge {

MaterialId MaterialDatabase::Register(const MaterialProperties& props) {
    const MaterialId id = static_cast<MaterialId>(materials.size());
    nameToId[props.name] = id;
    materials.push_back(props);
    return id;
}

const MaterialProperties& MaterialDatabase::Get(MaterialId id) const { return materials.at(id); }

MaterialId MaterialDatabase::FindByName(const std::string& name) const {
    auto it = nameToId.find(name);
    return it != nameToId.end() ? it->second : kInvalidMaterial;
}

size_t MaterialDatabase::Count() const { return materials.size(); }

MaterialDatabase MaterialDatabase::CreateDefault() {
    // Son iki alan: friction, restitution (gercek dunya yaklasik degerleri --
    // buz neredeyse surtunmesiz, kaucuk yuksek surtunme + belirgin sekme,
    // beton yuksek surtunme + olu temas).
    MaterialDatabase db;
    db.Register({"steel", SurfaceType::Metal, 7850.0f, 150.0f, 8.0f, 17.0f, 0.05f, 0.45f, 0.15f});
    db.Register(
        {"concrete", SurfaceType::Concrete, 2400.0f, 40.0f, 1.2f, 12.0f, 0.60f, 0.80f, 0.05f});
    db.Register({"wood", SurfaceType::Wood, 600.0f, 3.0f, 0.15f, 0.0f, 0.30f, 0.55f, 0.20f});
    db.Register({"soil", SurfaceType::Soil, 1600.0f, 1.0f, 0.10f, 0.0f, 0.00f, 0.70f, 0.02f});
    db.Register({"glass", SurfaceType::Glass, 2500.0f, 500.0f, 0.40f, 0.0f, 0.95f, 0.35f, 0.10f});
    db.Register({"flesh", SurfaceType::Flesh, 1050.0f, 1.0f, 0.07f, 0.0f, 0.00f, 0.60f, 0.05f});
    db.Register({"ice", SurfaceType::Custom, 917.0f, 5.0f, 0.08f, 0.0f, 0.40f, 0.05f, 0.05f});
    db.Register({"rubber", SurfaceType::Custom, 1100.0f, 2.0f, 0.05f, 0.0f, 0.00f, 1.0f, 0.75f});
    return db;
}

void MaterialDatabase::ApplyToBody(JPH::BodyInterface& bodyInterface, JPH::BodyID body,
                                   MaterialId id) const {
    const MaterialProperties& props = Get(id);
    bodyInterface.SetFriction(body, props.friction);
    bodyInterface.SetRestitution(body, props.restitution);
    bodyInterface.SetUserData(body, id);
}

} // namespace alazforge
