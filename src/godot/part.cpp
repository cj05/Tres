#include "part.hpp"
#include "modular_component.hpp"
#include "geometry_component.hpp"
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/classes/engine.hpp>

using namespace godot;

Part::Part() {}
Part::~Part() {}

void Part::_bind_methods() {
    ClassDB::bind_method(D_METHOD("set_mesh_path", "path"), &Part::set_mesh_path);
    ClassDB::bind_method(D_METHOD("get_mesh_path"), &Part::get_mesh_path);
    ClassDB::bind_method(D_METHOD("request_rebuild"), &Part::request_rebuild);
    ClassDB::bind_method(D_METHOD("get_components"), &Part::get_components);
    ClassDB::bind_method(D_METHOD("_rebuild_components"), &Part::_rebuild_components);
    ClassDB::bind_method(D_METHOD("get_total_mass"), &Part::get_total_mass);
    ClassDB::bind_method(D_METHOD("physics_step", "state"), &Part::physics_step);
    ClassDB::bind_method(D_METHOD("compute_total_force", "state"), &Part::compute_total_force);

    ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "mesh_path"), "set_mesh_path", "get_mesh_path");
}

void Part::set_mesh_path(const NodePath &p_path) {
    mesh_path = p_path;
}

NodePath Part::get_mesh_path() const {
    return mesh_path;
}

void Part::_ready() {
    _cache_mesh();
    _rebuild_components();
}

void Part::_cache_mesh() {
    if (!mesh_path.is_empty()) {
        mesh_instance = get_node<MeshInstance3D>(mesh_path);
    } else {
        for (int i = 0; i < get_child_count(); i++) {
            MeshInstance3D *child = Object::cast_to<MeshInstance3D>(get_child(i));
            if (child) {
                mesh_instance = child;
                break;
            }
        }
    }
}

void Part::request_rebuild() {
    if (_rebuild_requested) {
        return;
    }
    _rebuild_requested = true;
    call_deferred("_rebuild_components");
}

TypedArray<ModularComponent> Part::get_components() const {
    TypedArray<ModularComponent> result;
    for (int i = 0; i < get_child_count(); i++) {
        ModularComponent *component = Object::cast_to<ModularComponent>(get_child(i));
        if (component) {
            result.append(component);
        }
    }
    return result;
}

void Part::_rebuild_components() {
    if (_is_rebuilding) {
        return;
    }

    _is_rebuilding = true;
    _rebuild_requested = false;

    // Clear old collisions
    for (int i = 0; i < collision_shapes.size(); i++) {
        CollisionShape3D *cs = Object::cast_to<CollisionShape3D>(collision_shapes[i]);
        if (cs) {
            cs->queue_free();
        }
    }
    collision_shapes.clear();

    TypedArray<ModularComponent> components = get_components();
    for (int i = 0; i < components.size(); i++) {
        ModularComponent *component = Object::cast_to<ModularComponent>(components[i]);
        if (component && component->is_active()) {
            component->rebuild();

            GeometryComponent *geo = Object::cast_to<GeometryComponent>(component);
            if (geo) {
                Ref<Shape3D> shape = geo->get_shape();
                if (shape.is_valid()) {
                    CollisionShape3D *cs = memnew(CollisionShape3D);
                    cs->set_shape(shape);
                    add_child(cs);
                    collision_shapes.append(cs);
                }

                if (mesh_instance) {
                    Ref<Mesh> mesh = geo->get_mesh();
                    if (mesh.is_valid()) {
                        mesh_instance->set_mesh(mesh);
                    }
                }
            }
        }
    }

    _is_rebuilding = false;
}

double Part::get_total_mass() const {
    double total = 0.0;
    TypedArray<ModularComponent> components = get_components();
    for (int i = 0; i < components.size(); i++) {
        ModularComponent *component = Object::cast_to<ModularComponent>(components[i]);
        if (component && component->is_active()) {
            total += component->get_mass();
        }
    }
    return total;
}

void Part::physics_step(Variant p_state) {
    TypedArray<ModularComponent> components = get_components();
    for (int i = 0; i < components.size(); i++) {
        ModularComponent *component = Object::cast_to<ModularComponent>(components[i]);
        if (component && component->is_active()) {
            component->physics_step(p_state);
        }
    }
}

Vector3 Part::compute_total_force(Variant p_state) {
    Vector3 total;
    TypedArray<ModularComponent> components = get_components();
    for (int i = 0; i < components.size(); i++) {
        ModularComponent *component = Object::cast_to<ModularComponent>(components[i]);
        if (component && component->is_active()) {
            total += component->compute_force(p_state);
        }
    }
    return total;
}
