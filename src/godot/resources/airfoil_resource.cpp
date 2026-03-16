#include "airfoil_resource.hpp"
#include "../utils/airfoil_util.hpp"
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

using namespace godot;

AirfoilResource::AirfoilResource() {
    coefficients.instantiate();
}

AirfoilResource::~AirfoilResource() {}

void AirfoilResource::_bind_methods() {
    ClassDB::bind_method(D_METHOD("set_outline_points", "points"), &AirfoilResource::set_outline_points);
    ClassDB::bind_method(D_METHOD("get_outline_points"), &AirfoilResource::get_outline_points);
    ClassDB::bind_method(D_METHOD("set_coefficients", "coefficients"), &AirfoilResource::set_coefficients);
    ClassDB::bind_method(D_METHOD("get_coefficients"), &AirfoilResource::get_coefficients);
    ClassDB::bind_method(D_METHOD("set_paste_raw_data", "data"), &AirfoilResource::set_paste_raw_data);
    ClassDB::bind_method(D_METHOD("get_paste_raw_data"), &AirfoilResource::get_paste_raw_data);

    ClassDB::bind_method(D_METHOD("set_lift_curve_slope", "slope"), &AirfoilResource::set_lift_curve_slope);
    ClassDB::bind_method(D_METHOD("get_lift_curve_slope"), &AirfoilResource::get_lift_curve_slope);
    ClassDB::bind_method(D_METHOD("set_zero_lift_alpha", "alpha"), &AirfoilResource::set_zero_lift_alpha);
    ClassDB::bind_method(D_METHOD("get_zero_lift_alpha"), &AirfoilResource::get_zero_lift_alpha);
    ClassDB::bind_method(D_METHOD("set_cl_max", "cl"), &AirfoilResource::set_cl_max);
    ClassDB::bind_method(D_METHOD("get_cl_max"), &AirfoilResource::get_cl_max);

    ADD_GROUP("Visual Geometry", "");
    ADD_PROPERTY(PropertyInfo(Variant::PACKED_VECTOR2_ARRAY, "outline_points"), "set_outline_points", "get_outline_points");

    ADD_GROUP("Physics Data", "");
    ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "coefficients", PROPERTY_HINT_RESOURCE_TYPE, "CoefficientCurve"), "set_coefficients", "get_coefficients");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "lift_curve_slope"), "set_lift_curve_slope", "get_lift_curve_slope");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "zero_lift_alpha"), "set_zero_lift_alpha", "get_zero_lift_alpha");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "cl_max"), "set_cl_max", "get_cl_max");

    ADD_GROUP("Import Raw", "");
    ADD_PROPERTY(PropertyInfo(Variant::STRING, "paste_raw_data", PROPERTY_HINT_MULTILINE_TEXT), "set_paste_raw_data", "get_paste_raw_data");
}

void AirfoilResource::set_outline_points(const PackedVector2Array &p_points) {
    outline_points = p_points;
    emit_changed();
}

PackedVector2Array AirfoilResource::get_outline_points() const {
    return outline_points;
}

void AirfoilResource::set_coefficients(const Ref<CoefficientCurve> &p_coefficients) {
    if (p_coefficients.is_valid()) {
        coefficients = p_coefficients;
    }
    emit_changed();
}

Ref<CoefficientCurve> AirfoilResource::get_coefficients() const {
    if (coefficients.is_null()) {
        const_cast<AirfoilResource*>(this)->coefficients.instantiate();
    }
    return coefficients;
}

void AirfoilResource::set_paste_raw_data(const String &p_data) {
    if (paste_raw_data == p_data) {
        return;
    }
    paste_raw_data = p_data;
    if (paste_raw_data.is_empty()) {
        return;
    }

    _smart_parse(paste_raw_data);
    paste_raw_data = "";
    emit_changed();
}

String AirfoilResource::get_paste_raw_data() const {
    return paste_raw_data;
}

void AirfoilResource::_smart_parse(const String &p_data) {
    if (p_data.contains("XFOIL") || p_data.contains("Calculated polar")) {
        AirfoilUtil::import_from_text(p_data, Ref<AirfoilResource>(this));
    } else {
        _parse_shape_data(p_data);
    }
}

void AirfoilResource::set_lift_curve_slope(double p_slope) {
    lift_curve_slope = p_slope;
    emit_changed();
}

double AirfoilResource::get_lift_curve_slope() const {
    return lift_curve_slope;
}

void AirfoilResource::set_zero_lift_alpha(double p_alpha) {
    zero_lift_alpha = p_alpha;
    emit_changed();
}

double AirfoilResource::get_zero_lift_alpha() const {
    return zero_lift_alpha;
}

void AirfoilResource::set_cl_max(double p_cl) {
    cl_max = p_cl;
    emit_changed();
}

double AirfoilResource::get_cl_max() const {
    return cl_max;
}

void AirfoilResource::_parse_shape_data(const String &p_data) {
    PackedVector2Array new_points;
    PackedStringArray lines = p_data.split("\n", false);
    for (int i = 0; i < lines.size(); i++) {
        PackedStringArray parts = lines[i].split(" ", false);
        if (parts.size() >= 2) {
            new_points.append(Vector2(parts[0].to_float(), parts[1].to_float()));
        }
    }

    if (new_points.size() > 0) {
        outline_points = new_points;
    }
}
