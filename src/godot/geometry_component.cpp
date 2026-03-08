#include "geometry_component.hpp"
#include "part.hpp"
#include <godot_cpp/core/class_db.hpp>

using namespace godot;

GeometryComponent::GeometryComponent() {
    properties.instantiate();
}

GeometryComponent::~GeometryComponent() {}

void GeometryComponent::_bind_methods() {
    ClassDB::bind_method(D_METHOD("set_properties", "properties"), &GeometryComponent::set_properties);
    ClassDB::bind_method(D_METHOD("get_properties"), &GeometryComponent::get_properties);
    ClassDB::bind_method(D_METHOD("_notify_change"), &GeometryComponent::_notify_change);
    ClassDB::bind_method(D_METHOD("_generate_geometry"), &GeometryComponent::_generate_geometry);
    ClassDB::bind_method(D_METHOD("compute_volume"), &GeometryComponent::compute_volume);
    ClassDB::bind_method(D_METHOD("get_shape"), &GeometryComponent::get_shape);
    ClassDB::bind_method(D_METHOD("get_mesh"), &GeometryComponent::get_mesh);

    ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "properties", PROPERTY_HINT_RESOURCE_TYPE, "TooledProperties"), "set_properties", "get_properties");
}

void GeometryComponent::set_properties(const Ref<TooledProperties> &p_properties) {
    if (properties.is_valid()) {
        properties->disconnect("changed", Callable(this, "_notify_change"));
    }
    properties = p_properties;
    if (properties.is_valid()) {
        properties->connect("changed", Callable(this, "_notify_change"));
    }
    _notify_change();
}

Ref<TooledProperties> GeometryComponent::get_properties() const {
    return properties;
}

void GeometryComponent::rebuild() {
    _generate_geometry();
}

void GeometryComponent::_generate_geometry() {
}

double GeometryComponent::compute_volume() const {
    return 0.0;
}

void GeometryComponent::_notify_change() {
    Part *p = Object::cast_to<Part>(get_part());
    if (p) {
        p->request_rebuild();
    }
}

Ref<Shape3D> GeometryComponent::get_shape() const {
    return _shape;
}

Ref<Mesh> GeometryComponent::get_mesh() const {
    return _mesh;
}

double GeometryComponent::get_mass() const {
    if (properties.is_valid()) {
        if (properties->is_override_mass()) {
            return properties->get_custom_mass();
        }
        return compute_volume() * properties->get_density();
    }
    return 0.0;
}
