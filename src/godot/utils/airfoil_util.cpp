#include "airfoil_util.hpp"
#include "xfoil_parser.hpp"
#include "curve_prediction.hpp"
#include "../resources/airfoil_resource.hpp"

using namespace godot;

void AirfoilUtil::_bind_methods() {
    ClassDB::bind_static_method("AirfoilUtil", D_METHOD("import_from_text", "raw_text", "target"), &AirfoilUtil::import_from_text);
    ClassDB::bind_static_method("AirfoilUtil", D_METHOD("calculate_linear_regression", "alphas", "cls"), &AirfoilUtil::calculate_linear_regression);
}

void AirfoilUtil::import_from_text(const String &raw_text, const Ref<AirfoilResource> &target) {
    if (raw_text.is_empty()) {
        WARN_PRINT("AirfoilUtil: Input text is empty.");
        return;
    }

    if (target.is_null()) {
        ERR_PRINT("AirfoilUtil: Target AirfoilResource is missing.");
        return;
    }

    Ref<CoefficientCurve> coefficients = target->get_coefficients();
    if (coefficients.is_null()) {
        ERR_PRINT("AirfoilUtil: Target CoefficientCurve is missing.");
        return;
    }

    String clean_text = raw_text.replace("\r", "");

    if (_is_xfoil_format(clean_text)) {
        Dictionary points = XFoilParser::get_clean_points(clean_text);
        CurvePrediction::build_ideal_baseline(coefficients, points);

        // Extract scalar values for VLM
        PackedFloat32Array alphas = points["alpha"];
        PackedFloat32Array cls = points["cl"];

        if (alphas.size() > 0) {
            // 1. Cl,max
            float max_cl = -1e20f;
            for (int i = 0; i < cls.size(); i++) {
                if (cls[i] > max_cl) max_cl = cls[i];
            }
            target->set_cl_max(max_cl);

            // 2. Linear regression for slope and zero-lift alpha
            // Use the trimmed linear core for better regression
            Dictionary linear_data = CurvePrediction::get_trimmed_linear_core(points);
            PackedFloat32Array linear_alphas = linear_data["alpha"];
            PackedFloat32Array linear_cls = linear_data["cl"];

            Dictionary reg = calculate_linear_regression(linear_alphas, linear_cls);

            target->set_lift_curve_slope(reg["slope"]);
            target->set_zero_lift_alpha(reg["zero_lift_alpha"]);
        }

    } else {
        ERR_PRINT("AirfoilUtil: Format not recognized. Ensure the XFOIL header is included.");
    }
}

Dictionary AirfoilUtil::calculate_linear_regression(const PackedFloat32Array &alphas, const PackedFloat32Array &cls) {
    Dictionary res;
    res["slope"] = 0.0;
    res["zero_lift_alpha"] = 0.0;

    if (alphas.size() < 2 || alphas.size() != cls.size()) {
        return res;
    }

    double sum_x = 0, sum_y = 0, sum_xy = 0, sum_x2 = 0;
    int n = alphas.size();

    for (int i = 0; i < n; i++) {
        double x = alphas[i];
        double y = cls[i];
        sum_x += x;
        sum_y += y;
        sum_xy += x * y;
        sum_x2 += x * x;
    }

    double denominator = (n * sum_x2 - sum_x * sum_x);
    if (Math::abs(denominator) < 1e-9) {
        return res;
    }

    double slope = (n * sum_xy - sum_x * sum_y) / denominator;
    double intercept = (sum_y - slope * sum_x) / n;

    res["slope"] = slope;
    // Cl = slope * alpha + intercept
    // 0 = slope * zero_lift_alpha + intercept -> zero_lift_alpha = -intercept / slope
    if (Math::abs(slope) > 1e-9) {
        res["zero_lift_alpha"] = -intercept / slope;
    } else {
        res["zero_lift_alpha"] = 0.0;
    }

    return res;
}

bool AirfoilUtil::_is_xfoil_format(const String &text) {
    static const char *signatures[] = {
        "XFOIL",
        "Calculated polar for:",
        "Reynolds number fixed"
    };

    for (const char *sig : signatures) {
        if (text.contains(sig)) {
            return true;
        }
    }
    return false;
}
