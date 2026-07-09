#include "material_db/MaterialDatabase.h"

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
    MaterialDatabase db;
    db.Register({"steel", SurfaceType::Metal, 7850.0f, 150.0f, 8.0f, 17.0f, 0.05f});
    db.Register({"concrete", SurfaceType::Concrete, 2400.0f, 40.0f, 1.2f, 12.0f, 0.60f});
    db.Register({"wood", SurfaceType::Wood, 600.0f, 3.0f, 0.15f, 0.0f, 0.30f});
    db.Register({"soil", SurfaceType::Soil, 1600.0f, 1.0f, 0.10f, 0.0f, 0.00f});
    db.Register({"glass", SurfaceType::Glass, 2500.0f, 500.0f, 0.40f, 0.0f, 0.95f});
    db.Register({"flesh", SurfaceType::Flesh, 1050.0f, 1.0f, 0.07f, 0.0f, 0.00f});
    return db;
}

} // namespace alazforge
