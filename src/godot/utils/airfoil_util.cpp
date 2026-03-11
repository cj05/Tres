#include "airfoil_util.hpp"
#include "xfoil_parser.hpp"
#include "curve_prediction.hpp"

using namespace godot;

void AirfoilUtil::_bind_methods() {
    ClassDB::bind_static_method("AirfoilUtil", D_METHOD("import_from_text", "raw_text", "target"), &AirfoilUtil::import_from_text);
}

void AirfoilUtil::import_from_text(const String &raw_text, const Ref<CoefficientCurve> &target) {
    if (raw_text.is_empty()) {
        WARN_PRINT("AirfoilUtil: Input text is empty.");
        return;
    }

    if (target.is_null()) {
        ERR_PRINT("AirfoilUtil: Target CoefficientCurve is missing.");
        return;
    }

    String clean_text = raw_text.replace("\r", "");

    if (_is_xfoil_format(clean_text)) {
        Dictionary points = XFoilParser::get_clean_points(clean_text);
        CurvePrediction::build_ideal_baseline(target, points);
    } else {
        ERR_PRINT("AirfoilUtil: Format not recognized. Ensure the XFOIL header is included.");
    }
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
