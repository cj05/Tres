#include "tooled_properties.hpp"
#include <godot_cpp/core/class_db.hpp>

using namespace godot;

TooledProperties::TooledProperties() {}
TooledProperties::~TooledProperties() {}

void TooledProperties::_bind_methods() {
    ClassDB::bind_method(D_METHOD("set_density", "density"), &TooledProperties::set_density);
    ClassDB::bind_method(D_METHOD("get_density"), &TooledProperties::get_density);
    ClassDB::bind_method(D_METHOD("set_override_mass", "override_mass"), &TooledProperties::set_override_mass);
    ClassDB::bind_method(D_METHOD("is_override_mass"), &TooledProperties::is_override_mass);
    ClassDB::bind_method(D_METHOD("set_custom_mass", "custom_mass"), &TooledProperties::set_custom_mass);
    ClassDB::bind_method(D_METHOD("get_custom_mass"), &TooledProperties::get_custom_mass);

    ADD_GROUP("General", "");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "density"), "set_density", "get_density");
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "override_mass"), "set_override_mass", "is_override_mass");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "custom_mass"), "set_custom_mass", "get_custom_mass");
}

void TooledProperties::set_density(double p_density) {
    density = MAX(p_density, 0.0001);
    emit_changed();
}

double TooledProperties::get_density() const {
    return density;
}

void TooledProperties::set_override_mass(bool p_override) {
    override_mass = p_override;
    emit_changed();
}

bool TooledProperties::is_override_mass() const {
    return override_mass;
}

void TooledProperties::set_custom_mass(double p_mass) {
    custom_mass = MAX(p_mass, 0.0);
    emit_changed();
}

double TooledProperties::get_custom_mass() const {
    return custom_mass;
}
