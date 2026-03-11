#include "aero_geometry_properties.hpp"
#include <godot_cpp/core/class_db.hpp>

using namespace godot;

AeroGeometryProperties::AeroGeometryProperties() {}
AeroGeometryProperties::~AeroGeometryProperties() {}

void AeroGeometryProperties::_bind_methods() {
    ClassDB::bind_method(D_METHOD("set_strength", "strength"), &AeroGeometryProperties::set_strength);
    ClassDB::bind_method(D_METHOD("get_strength"), &AeroGeometryProperties::get_strength);
    ClassDB::bind_method(D_METHOD("set_sections", "sections"), &AeroGeometryProperties::set_sections);
    ClassDB::bind_method(D_METHOD("get_sections"), &AeroGeometryProperties::get_sections);
    ClassDB::bind_method(D_METHOD("set_root_chord", "chord"), &AeroGeometryProperties::set_root_chord);
    ClassDB::bind_method(D_METHOD("get_root_chord"), &AeroGeometryProperties::get_root_chord);
    ClassDB::bind_method(D_METHOD("set_root_airfoil", "airfoil"), &AeroGeometryProperties::set_root_airfoil);
    ClassDB::bind_method(D_METHOD("get_root_airfoil"), &AeroGeometryProperties::get_root_airfoil);
    ClassDB::bind_method(D_METHOD("_on_sub_resource_changed"), &AeroGeometryProperties::_on_sub_resource_changed);

    ADD_GROUP("AeroSurfaceMesh", "");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "strength"), "set_strength", "get_strength");
    ADD_PROPERTY(PropertyInfo(Variant::ARRAY,"sections",PROPERTY_HINT_ARRAY_TYPE,"WingSection"), "set_sections", "get_sections");

    ADD_GROUP("RootProperties", "");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "root_chord"), "set_root_chord", "get_root_chord");
    ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "root_airfoil", PROPERTY_HINT_RESOURCE_TYPE, "AirfoilResource"), "set_root_airfoil", "get_root_airfoil");
}

void AeroGeometryProperties::set_strength(double p_strength) {
    strength = p_strength;
    emit_changed();
}

double AeroGeometryProperties::get_strength() const {
    return strength;
}

void AeroGeometryProperties::set_sections(const TypedArray<WingSection> &p_sections) {
    for (int i = 0; i < sections.size(); i++) {
        Ref<WingSection> s = sections[i];
        if (s.is_valid()) {
            s->disconnect("changed", Callable(this, "_on_sub_resource_changed"));
        }
    }
    sections = p_sections;
    for (int i = 0; i < sections.size(); i++) {
        Ref<WingSection> s = sections[i];
        if (s.is_valid()) {
            if (!s->is_connected("changed", Callable(this, "_on_sub_resource_changed"))) {
                s->connect("changed", Callable(this, "_on_sub_resource_changed"));
            }
        }
    }
    emit_changed();
}

TypedArray<WingSection> AeroGeometryProperties::get_sections() const {
    return sections;
}

void AeroGeometryProperties::set_root_chord(double p_chord) {
    root_chord = p_chord;
    emit_changed();
}

double AeroGeometryProperties::get_root_chord() const {
    return root_chord;
}

void AeroGeometryProperties::set_root_airfoil(const Ref<AirfoilResource> &p_airfoil) {
    root_airfoil = p_airfoil;
    emit_changed();
}

Ref<AirfoilResource> AeroGeometryProperties::get_root_airfoil() const {
    return root_airfoil;
}

void AeroGeometryProperties::_on_sub_resource_changed() {
    emit_changed();
}
