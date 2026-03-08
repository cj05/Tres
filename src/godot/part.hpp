#pragma once

#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/classes/mesh_instance3d.hpp>
#include <godot_cpp/classes/collision_shape3d.hpp>
#include <godot_cpp/classes/shape3d.hpp>
#include <godot_cpp/templates/vector.hpp>

namespace godot {

class ModularComponent;

class Part : public Node3D {
    GDCLASS(Part, Node3D);

protected:
    static void _bind_methods();

    NodePath mesh_path;
    MeshInstance3D *mesh_instance = nullptr;
    TypedArray<CollisionShape3D> collision_shapes;

    bool _rebuild_requested = false;
    bool _is_rebuilding = false;

    void _cache_mesh();
    void _rebuild_components();

public:
    Part();
    ~Part();

    void set_mesh_path(const NodePath &p_path);
    NodePath get_mesh_path() const;

    void _ready() override;

    void request_rebuild();
    TypedArray<ModularComponent> get_components() const;

    double get_total_mass() const;
    void physics_step(Variant p_state);
    Vector3 compute_total_force(Variant p_state);
};

} // namespace godot
