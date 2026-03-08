#include "coefficient_curve.hpp"
#include <godot_cpp/core/class_db.hpp>

using namespace godot;

CoefficientCurve::CoefficientCurve() {
    lift_ideal_curve.instantiate();
    drag_ideal_curve.instantiate();
    moment_ideal_curve.instantiate();
}

CoefficientCurve::~CoefficientCurve() {}

void CoefficientCurve::_bind_methods() {
    ClassDB::bind_method(D_METHOD("set_lift_ideal_curve", "curve"), &CoefficientCurve::set_lift_ideal_curve);
    ClassDB::bind_method(D_METHOD("get_lift_ideal_curve"), &CoefficientCurve::get_lift_ideal_curve);
    ClassDB::bind_method(D_METHOD("set_drag_ideal_curve", "curve"), &CoefficientCurve::set_drag_ideal_curve);
    ClassDB::bind_method(D_METHOD("get_drag_ideal_curve"), &CoefficientCurve::get_drag_ideal_curve);
    ClassDB::bind_method(D_METHOD("set_moment_ideal_curve", "curve"), &CoefficientCurve::set_moment_ideal_curve);
    ClassDB::bind_method(D_METHOD("get_moment_ideal_curve"), &CoefficientCurve::get_moment_ideal_curve);
    ClassDB::bind_method(D_METHOD("sample_all", "alpha_deg"), &CoefficientCurve::sample_all);
    ClassDB::bind_method(D_METHOD("clear_all"), &CoefficientCurve::clear_all);

    ADD_GROUP("Curves", "");
    ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "lift_ideal_curve", PROPERTY_HINT_RESOURCE_TYPE, "Curve"), "set_lift_ideal_curve", "get_lift_ideal_curve");
    ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "drag_ideal_curve", PROPERTY_HINT_RESOURCE_TYPE, "Curve"), "set_drag_ideal_curve", "get_drag_ideal_curve");
    ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "moment_ideal_curve", PROPERTY_HINT_RESOURCE_TYPE, "Curve"), "set_moment_ideal_curve", "get_moment_ideal_curve");
}

void CoefficientCurve::set_lift_ideal_curve(const Ref<Curve> &p_curve) {
    lift_ideal_curve = p_curve;
}

Ref<Curve> CoefficientCurve::get_lift_ideal_curve() const {
    return lift_ideal_curve;
}

void CoefficientCurve::set_drag_ideal_curve(const Ref<Curve> &p_curve) {
    drag_ideal_curve = p_curve;
}

Ref<Curve> CoefficientCurve::get_drag_ideal_curve() const {
    return drag_ideal_curve;
}

void CoefficientCurve::set_moment_ideal_curve(const Ref<Curve> &p_curve) {
    moment_ideal_curve = p_curve;
}

Ref<Curve> CoefficientCurve::get_moment_ideal_curve() const {
    return moment_ideal_curve;
}

Dictionary CoefficientCurve::sample_all(double alpha_deg) const {
    Dictionary res;
    res["cl"] = lift_ideal_curve->sample_baked(alpha_deg);
    res["cd"] = drag_ideal_curve->sample_baked(alpha_deg);
    res["cm"] = moment_ideal_curve->sample_baked(alpha_deg);
    return res;
}

void CoefficientCurve::clear_all() {
    lift_ideal_curve->clear_points();
    drag_ideal_curve->clear_points();
    moment_ideal_curve->clear_points();

    double max_angle = 90.0;
    double max_coef = 5.0;

    auto setup_curve = [&](Ref<Curve> c) {
        // In Godot 4.x Curve doesn't have min_domain/max_domain/min_value/max_value like in GDScript snippet maybe?
        // Actually it has min_value and max_value.
        c->set_min_value(-max_coef);
        c->set_max_value(max_coef);
    };

    setup_curve(lift_ideal_curve);
    setup_curve(drag_ideal_curve);
    setup_curve(moment_ideal_curve);
}
