#pragma once

#include <godot_cpp/classes/resource.hpp>
#include "coefficient_curve.hpp"

namespace godot {

class AirfoilResource : public Resource {
    GDCLASS(AirfoilResource, Resource);

protected:
    static void _bind_methods();

    PackedVector2Array outline_points;
    Ref<CoefficientCurve> coefficients;
    String paste_raw_data;

    double lift_curve_slope = 0.0;
    double zero_lift_alpha = 0.0;
    double cl_max = 0.0;

    void _smart_parse(const String &p_data);
    void _parse_shape_data(const String &p_data);

public:
    AirfoilResource();
    ~AirfoilResource();

    void set_outline_points(const PackedVector2Array &p_points);
    PackedVector2Array get_outline_points() const;

    void set_coefficients(const Ref<CoefficientCurve> &p_coefficients);
    Ref<CoefficientCurve> get_coefficients() const;

    void set_paste_raw_data(const String &p_data);
    String get_paste_raw_data() const;

    void set_lift_curve_slope(double p_slope);
    double get_lift_curve_slope() const;

    void set_zero_lift_alpha(double p_alpha);
    double get_zero_lift_alpha() const;

    void set_cl_max(double p_cl);
    double get_cl_max() const;
};

} // namespace godot
