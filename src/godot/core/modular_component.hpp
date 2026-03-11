#pragma once

#include <godot_cpp/classes/node3d.hpp>

namespace godot {

class ModularComponent : public Node3D {
    GDCLASS(ModularComponent, Node3D);

protected:
    static void _bind_methods();

    bool enabled = true;
    Node *part = nullptr;

public:
    ModularComponent();
    ~ModularComponent();

    void set_enabled(bool p_enabled);
    bool is_enabled() const;

    void _ready() override;
    void _exit_tree() override;

    virtual void _on_component_ready();
    virtual void _on_component_removed();
    virtual void _on_enabled_changed();

    bool is_active() const;
    Node *get_part() const;

    virtual void physics_step(Variant p_state);
    virtual void rebuild();
    virtual double get_mass() const;
    virtual Vector3 compute_force(Variant p_state);
};

} // namespace godot
