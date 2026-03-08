#pragma once

#include <godot_cpp/core/class_db.hpp>
#include "coefficient_curve.hpp"

namespace godot {

class AirfoilUtil : public Object {
    GDCLASS(AirfoilUtil, Object);

protected:
    static void _bind_methods();
    static bool _is_xfoil_format(const String &text);

public:
    static void import_from_text(const String &raw_text, const Ref<CoefficientCurve> &target);
};

} // namespace godot
