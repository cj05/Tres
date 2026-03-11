#pragma once

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include "../resources/coefficient_curve.hpp"

namespace godot {

class CurvePrediction : public Object {
    GDCLASS(CurvePrediction, Object);

protected:
    static void _bind_methods();

    static Dictionary _get_trimmed_linear_core(const Dictionary &data);
    static Dictionary _slice_range(const Dictionary &data, int start, int end);
    static int _find_closest_index(const PackedFloat32Array &arr, float target);
    static int _find_permanent_deviation(const PackedFloat32Array &alphas, const PackedFloat32Array &cls, int start_idx, float ref_slope, int direction);
    static float _find_deep_stall(const PackedFloat32Array &alphas, const PackedFloat32Array &cls, int stall_start_idx, int direction = 1);
    static void _apply_idealized_extensions(const Ref<CoefficientCurve> &target, const Dictionary &data);

public:
    static void build_ideal_baseline(const Ref<CoefficientCurve> &target, const Dictionary &clean_data);
};

} // namespace godot
