#include "modular_component.hpp"
#include <godot_cpp/core/class_db.hpp>

using namespace godot;

ModularComponent::ModularComponent() {}
ModularComponent::~ModularComponent() {}

void ModularComponent::_bind_methods() {
    ClassDB::bind_method(D_METHOD("set_enabled", "enabled"), &ModularComponent::set_enabled);
    ClassDB::bind_method(D_METHOD("is_enabled"), &ModularComponent::is_enabled);
    ClassDB::bind_method(D_METHOD("is_active"), &ModularComponent::is_active);
    ClassDB::bind_method(D_METHOD("get_part"), &ModularComponent::get_part);

    ClassDB::bind_method(D_METHOD("_on_component_ready"), &ModularComponent::_on_component_ready);
    ClassDB::bind_method(D_METHOD("_on_component_removed"), &ModularComponent::_on_component_removed);
    ClassDB::bind_method(D_METHOD("_on_enabled_changed"), &ModularComponent::_on_enabled_changed);

    ClassDB::bind_method(D_METHOD("physics_step", "state"), &ModularComponent::physics_step);
    ClassDB::bind_method(D_METHOD("rebuild"), &ModularComponent::rebuild);
    ClassDB::bind_method(D_METHOD("get_mass"), &ModularComponent::get_mass);
    ClassDB::bind_method(D_METHOD("compute_force", "state"), &ModularComponent::compute_force);

    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "enabled"), "set_enabled", "is_enabled");
}

void ModularComponent::set_enabled(bool p_enabled) {
    enabled = p_enabled;
    if (is_inside_tree()) {
        _on_enabled_changed();
    }
}

bool ModularComponent::is_enabled() const {
    return enabled;
}

void ModularComponent::_ready() {
    part = get_parent();
    if (part == nullptr) {
        WARN_PRINT("ModularComponent has no parent Part.");
    }
    _on_component_ready();
}

void ModularComponent::_exit_tree() {
    _on_component_removed();
}

void ModularComponent::_on_component_ready() {
}

void ModularComponent::_on_component_removed() {
}

void ModularComponent::_on_enabled_changed() {
}

bool ModularComponent::is_active() const {
    return enabled && is_inside_tree();
}

Node *ModularComponent::get_part() const {
    return part;
}

void ModularComponent::physics_step(Variant p_state) {
}

void ModularComponent::rebuild() {
}

double ModularComponent::get_mass() const {
    return 0.0;
}

Vector3 ModularComponent::compute_force(Variant p_state) {
    return Vector3();
}
