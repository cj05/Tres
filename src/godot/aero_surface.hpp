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
        double lift = 0.0;
        double drag = 0.0;
        double cl = 0.0;
        double cd = 0.0;
        Vector3 lift_vector;
        Vector3 drag_vector;
    };

    Vector<SubSection> subsections;
    double segments_per_meter = 4.0;
    Vector3 wind_velocity = Vector3(20, 0, 0);
    bool debug_draw = true;
    bool vlm_enabled = true;
    double debug_force_scale = 0.001; 
    double debug_vortex_scale = 0.5;

    struct Vortex {
        Vector3 left_tip;
        Vector3 right_tip;
        Vector3 collocation_point;
        Vector3 normal;
        double span;
        double circulation = 0.0;
        double lift = 0.0;
        double cl = 0.0;
        Vector3 lift_vector;
    };

    Vector<Vortex> vortices;

    MeshInstance3D *debug_mesh_instance = nullptr;
    Ref<ImmediateMesh> debug_mesh;
    Ref<StandardMaterial3D> debug_material;

    void _generate_subsections();
    void _update_vortices();
    void _solve_vlm();
    void _update_debug_draw();

    Vector3 _calculate_induced_velocity(const Vector3 &p_point, const Vector3 &p_v1, const Vector3 &p_v2, double p_gamma) const;
    Vector3 _calculate_horseshoe_velocity(const Vector3 &p_point, const Vortex &p_vortex, double p_gamma, const Vector3 &p_wind_dir) const;

public:
    AeroSurface();
    ~AeroSurface();

    void set_debug_draw(bool p_debug);
    bool is_debug_draw() const;

    void set_segments_per_meter(double p_segments);
    double get_segments_per_meter() const;

    void set_wind_velocity(Vector3 p_wind);
    Vector3 get_wind_velocity() const;

    void set_vlm_enabled(bool p_enabled);
    bool is_vlm_enabled() const;

    void set_debug_force_scale(double p_scale);
    double get_debug_force_scale() const;

    void _on_component_ready() override;
    void physics_step(Variant p_state) override;
    Vector3 compute_force(Variant p_state) override;

    void _process(double delta) override;
};

} // namespace godot
