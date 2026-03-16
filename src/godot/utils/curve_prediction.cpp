#include "curve_prediction.hpp"
#include <godot_cpp/variant/utility_functions.hpp>

using namespace godot;

void CurvePrediction::_bind_methods() {
    ClassDB::bind_static_method("CurvePrediction", D_METHOD("build_ideal_baseline", "target", "clean_data"), &CurvePrediction::build_ideal_baseline);
}

void CurvePrediction::build_ideal_baseline(const Ref<CoefficientCurve> &target, const Dictionary &clean_data) {
    Dictionary data = get_trimmed_linear_core(clean_data);

    target->clear_all();
    PackedFloat32Array alphas = data["alpha"];
    PackedFloat32Array cls = data["cl"];
    PackedFloat32Array cds = data["cd"];
    PackedFloat32Array cms = data["cm"];

    for (int i = 0; i < alphas.size(); i++) {
        target->get_lift_ideal_curve()->add_point(Vector2(alphas[i], cls[i]));
        target->get_drag_ideal_curve()->add_point(Vector2(alphas[i], cds[i]));
        target->get_moment_ideal_curve()->add_point(Vector2(alphas[i], cms[i]));
    }

    _apply_idealized_extensions(target, data);
}

void CurvePrediction::_apply_idealized_extensions(const Ref<CoefficientCurve> &target, const Dictionary &data) {
    // Ported from GDScript: print(data.p_stall_start,data._stall_start)
    // Note: GDScript had a typo _stall_start which I'll ignore or fix if I can find it.
}

Dictionary CurvePrediction::get_trimmed_linear_core(const Dictionary &data) {
    PackedFloat32Array alphas = data["alpha"];
    PackedFloat32Array cls = data["cl"];

    int i_ref_low = _find_closest_index(alphas, 0.0);
    int i_ref_high = _find_closest_index(alphas, 4.0);
    float ref_slope = (cls[i_ref_high] - cls[i_ref_low]) / (alphas[i_ref_high] - alphas[i_ref_low]);

    int p_stall_start_idx = _find_permanent_deviation(alphas, cls, i_ref_high, ref_slope, 1);
    float p_deep_stall = _find_deep_stall(alphas, cls, p_stall_start_idx);

    int n_stall_start_idx = _find_permanent_deviation(alphas, cls, i_ref_low, ref_slope, -1);
    float n_deep_stall = _find_deep_stall(alphas, cls, n_stall_start_idx, -1);

    Dictionary out_dict = _slice_range(data, n_stall_start_idx, p_stall_start_idx);
    out_dict["p_stall_start"] = alphas[p_stall_start_idx];
    out_dict["p_deep_stall"] = p_deep_stall;
    out_dict["n_stall_start"] = alphas[n_stall_start_idx];
    out_dict["n_deep_stall"] = n_deep_stall;

    return out_dict;
}

Dictionary CurvePrediction::_get_trimmed_linear_core(const Dictionary &data) {
    return get_trimmed_linear_core(data);
}

Dictionary CurvePrediction::_slice_range(const Dictionary &data, int start, int end) {
    PackedFloat32Array alphas = data["alpha"];
    PackedFloat32Array cls = data["cl"];
    PackedFloat32Array cds = data["cd"];
    PackedFloat32Array cms = data["cm"];

    PackedFloat32Array s_alphas, s_cls, s_cds, s_cms;
    for (int i = start; i <= end; i++) {
        s_alphas.append(alphas[i]);
        s_cls.append(cls[i]);
        s_cds.append(cds[i]);
        s_cms.append(cms[i]);
    }

    Dictionary sliced;
    sliced["alpha"] = s_alphas;
    sliced["cl"] = s_cls;
    sliced["cd"] = s_cds;
    sliced["cm"] = s_cms;
    return sliced;
}

int CurvePrediction::_find_closest_index(const PackedFloat32Array &arr, float target) {
    int closest_idx = 0;
    float min_diff = 1e20f;
    for (int i = 0; i < arr.size(); i++) {
        float diff = Math::abs(arr[i] - target);
        if (diff < min_diff) {
            min_diff = diff;
            closest_idx = i;
        }
    }
    return closest_idx;
}

int CurvePrediction::_find_permanent_deviation(const PackedFloat32Array &alphas, const PackedFloat32Array &cls, int start_idx, float ref_slope, int direction) {
    int window_size = 8;
    int limit = (direction > 0) ? (cls.size() - window_size) : window_size;

    for (int i = start_idx; (direction > 0 ? i < limit : i > limit); i += direction) {
        bool is_permanent = true;
        for (int j = 1; j <= window_size; j++) {
            int check_idx = i + (j * direction);
            float local_slope = (cls[check_idx] - cls[i]) / (alphas[check_idx] - alphas[i]);
            if (local_slope > (ref_slope * 0.4f)) {
                is_permanent = false;
                break;
            }
        }
        if (is_permanent) {
            return i;
        }
    }
    return (direction > 0) ? start_idx : 0;
}

float CurvePrediction::_find_deep_stall(const PackedFloat32Array &alphas, const PackedFloat32Array &cls, int stall_start_idx, int direction) {
    int peak_idx = stall_start_idx;
    float max_cl = (direction > 0) ? -1e20f : 1e20f;

    int limit = (direction > 0) ? (cls.size() - 1) : 1;
    for (int i = stall_start_idx; (direction > 0 ? i < limit : i > limit); i += direction) {
        if ((direction > 0 && cls[i] > max_cl) || (direction < 0 && cls[i] < max_cl)) {
            max_cl = cls[i];
            peak_idx = i;
        }
        if ((direction > 0 && cls[i] < max_cl * 0.95f) || (direction < 0 && cls[i] > max_cl * 0.95f)) {
            break;
        }
    }

    if (peak_idx <= 0 || peak_idx >= cls.size() - 1) {
        return alphas[peak_idx];
    }

    float x1 = alphas[peak_idx - 1];
    float y1 = cls[peak_idx - 1];
    float x2 = alphas[peak_idx];
    float y2 = cls[peak_idx];
    float x3 = alphas[peak_idx + 1];
    float y3 = cls[peak_idx + 1];

    float denom = (x1 - x2) * (x1 - x3) * (x2 - x3);
    if (Math::abs(denom) < 1e-6f) return x2;

    float b = (x3 * x3 * (y1 - y2) + x2 * x2 * (y3 - y1) + x1 * x1 * (y2 - y3)) / denom;
    float a = (x3 * (y2 - y1) + x2 * (y1 - y3) + x1 * (y3 - y2)) / denom;

    if (Math::abs(a) < 1e-6f) return x2;

    float precise_stall_alpha = -b / (2.0f * a);
    return CLAMP(precise_stall_alpha, x1, x3);
}
