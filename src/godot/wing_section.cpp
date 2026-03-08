#include "wing_section.hpp"
#include <godot_cpp/core/class_db.hpp>

using namespace godot;

WingSection::WingSection() {}
WingSection::~WingSection() {}

void WingSection::_bind_methods() {
    ClassDB::bind_method(D_METHOD("set_span_offset", "offset"), &WingSection::set_span_offset);
    ClassDB::bind_method(D_METHOD("get_span_offset"), &WingSection::get_span_offset);
    ClassDB::bind_method(D_METHOD("set_sweep_offset", "offset"), &WingSection::set_sweep_offset);
    ClassDB::bind_method(D_METHOD("get_sweep_offset"), &WingSection::get_sweep_offset);
    ClassDB::bind_method(D_METHOD("set_dihedral_angle", "angle"), &WingSection::set_dihedral_angle);
    ClassDB::bind_method(D_METHOD("get_dihedral_angle"), &WingSection::get_dihedral_angle);
    ClassDB::bind_method(D_METHOD("set_chord", "chord"), &WingSection::set_chord);
    ClassDB::bind_method(D_METHOD("get_chord"), &WingSection::get_chord);
    ClassDB::bind_method(D_METHOD("set_twist", "twist"), &WingSection::set_twist);
    ClassDB::bind_method(D_METHOD("get_twist"), &WingSection::get_twist);
    ClassDB::bind_method(D_METHOD("set_airfoil_data", "data"), &WingSection::set_airfoil_data);
    ClassDB::bind_method(D_METHOD("get_airfoil_data"), &WingSection::get_airfoil_data);

    ADD_GROUP("Positioning", "");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "span_offset"), "set_span_offset", "get_span_offset");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "sweep_offset"), "set_sweep_offset", "get_sweep_offset");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "dihedral_angle"), "set_dihedral_angle", "get_dihedral_angle");

    ADD_GROUP("Aerodynamics", "");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "chord"), "set_chord", "get_chord");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "twist"), "set_twist", "get_twist");
    ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "airfoil_data", PROPERTY_HINT_RESOURCE_TYPE, "AirfoilResource"), "set_airfoil_data", "get_airfoil_data");
}

void WingSection::set_span_offset(double p_offset) {
    span_offset = p_offset;
    emit_changed();
}

double WingSection::get_span_offset() const {
    return span_offset;
}

void WingSection::set_sweep_offset(double p_offset) {
    sweep_offset = p_offset;
    emit_changed();
}

double WingSection::get_sweep_offset() const {
    return sweep_offset;
}

void WingSection::set_dihedral_angle(double p_angle) {
    dihedral_angle = p_angle;
    emit_changed();
}

double WingSection::get_dihedral_angle() const {
    return dihedral_angle;
}

void WingSection::set_chord(double p_chord) {
    chord = p_chord;
    emit_changed();
}

double WingSection::get_chord() const {
    return chord;
}

void WingSection::set_twist(double p_twist) {
    twist = p_twist;
    emit_changed();
}

double WingSection::get_twist() const {
    return twist;
}

void WingSection::set_airfoil_data(const Ref<AirfoilResource> &p_data) {
    airfoil_data = p_data;
    emit_changed();
}

Ref<AirfoilResource> WingSection::get_airfoil_data() const {
    return airfoil_data;
}
