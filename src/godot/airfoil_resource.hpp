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
};

} // namespace godot
