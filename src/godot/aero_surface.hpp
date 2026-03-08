#pragma once

#include "aero_component.hpp"
#include "aero_geometry.hpp"
#include "aero/flatplate_model.hpp"
#include <godot_cpp/classes/mesh_instance3d.hpp>
#include <godot_cpp/classes/immediate_mesh.hpp>
#include <godot_cpp/classes/standard_material3d.hpp>

namespace godot {

class AeroSurface : public AeroComponent {
    GDCLASS(AeroSurface, AeroComponent);

protected:
    static void _bind_methods();

    struct SubSection {
        Transform3D transform;
        double area;
        double chord;
        Ref<AirfoilResource> airfoil;
    };

    Vector<SubSection> subsections;
    double segments_per_meter = 4.0;
    Vector3 wind_velocity = Vector3(20, 0, 0);
    bool debug_draw = true;

    MeshInstance3D *debug_mesh_instance = nullptr;
    Ref<ImmediateMesh> debug_mesh;
    Ref<StandardMaterial3D> debug_material;

    void _generate_subsections();
    void _update_debug_draw();

public:
    AeroSurface();
    ~AeroSurface();

    void set_debug_draw(bool p_debug);
    bool is_debug_draw() const;

    void set_segments_per_meter(double p_segments);
    double get_segments_per_meter() const;

    void set_wind_velocity(Vector3 p_wind);
    Vector3 get_wind_velocity() const;

    void _on_component_ready() override;
    void physics_step(Variant p_state) override;
    Vector3 compute_force(Variant p_state) override;

    void _process(double delta) override;
};

} // namespace godot
