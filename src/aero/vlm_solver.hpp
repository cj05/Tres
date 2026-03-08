#pragma once

#include <godot_cpp/variant/vector3.hpp>
#include <godot_cpp/templates/vector.hpp>

namespace godot {

struct VLMPanel {
    Vector3 left_tip;
    Vector3 right_tip;
    Vector3 collocation_point;
    Vector3 normal;
    double area;
    double chord;
    double circulation = 0.0;
};

class VLMSolver {
public:
    static void solve(Vector<VLMPanel> &p_panels, const Vector3 &p_wind_velocity);

private:
    static Vector3 _calculate_induced_velocity(const Vector3 &p_point, const Vector3 &p_v1, const Vector3 &p_v2, double p_gamma);
    static Vector3 _calculate_horseshoe_velocity(const Vector3 &p_point, const VLMPanel &p_panel, double p_gamma, const Vector3 &p_wind_dir);
};

} // namespace godot
