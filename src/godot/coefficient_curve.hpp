#pragma once

#include <godot_cpp/classes/resource.hpp>
#include <godot_cpp/classes/curve.hpp>

namespace godot {

class CoefficientCurve : public Resource {
    GDCLASS(CoefficientCurve, Resource);

protected:
    static void _bind_methods();

    Ref<Curve> lift_ideal_curve;
    Ref<Curve> drag_ideal_curve;
    Ref<Curve> moment_ideal_curve;

public:
    CoefficientCurve();
    ~CoefficientCurve();

    void set_lift_ideal_curve(const Ref<Curve> &p_curve);
    Ref<Curve> get_lift_ideal_curve() const;

    void set_drag_ideal_curve(const Ref<Curve> &p_curve);
    Ref<Curve> get_drag_ideal_curve() const;

    void set_moment_ideal_curve(const Ref<Curve> &p_curve);
    Ref<Curve> get_moment_ideal_curve() const;

    Dictionary sample_all(double alpha_deg) const;
    void clear_all();
};

} // namespace godot
