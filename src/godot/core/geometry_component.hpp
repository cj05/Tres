#pragma once

#include "modular_component.hpp"
#include "../utils/tooled_properties.hpp"
#include <godot_cpp/classes/shape3d.hpp>
#include <godot_cpp/classes/mesh.hpp>

namespace godot {

class GeometryComponent : public ModularComponent {
    GDCLASS(GeometryComponent, ModularComponent);

protected:
    static void _bind_methods();

    Ref<TooledProperties> properties;
    Ref<Shape3D> _shape;
    Ref<Mesh> _mesh;

    void _notify_change();

public:
    GeometryComponent();
    ~GeometryComponent();

    void set_properties(const Ref<TooledProperties> &p_properties);
    Ref<TooledProperties> get_properties() const;

    void rebuild() override;
    virtual void _generate_geometry();
    virtual double compute_volume() const;

    Ref<Shape3D> get_shape() const;
    Ref<Mesh> get_mesh() const;

    double get_mass() const override;
};

} // namespace godot
