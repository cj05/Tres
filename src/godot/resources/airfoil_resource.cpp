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

    ADD_GROUP("Visual Geometry", "");
    ADD_PROPERTY(PropertyInfo(Variant::PACKED_VECTOR2_ARRAY, "outline_points"), "set_outline_points", "get_outline_points");

    ADD_GROUP("Physics Data", "");
    ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "coefficients", PROPERTY_HINT_RESOURCE_TYPE, "CoefficientCurve"), "set_coefficients", "get_coefficients");

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
        AirfoilUtil::import_from_text(p_data, coefficients);
    } else {
        _parse_shape_data(p_data);
    }
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
